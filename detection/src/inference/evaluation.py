# coding=utf-8
"""
written by ryanreadbooks
date: 2021/11/26
"""

import numpy as np
from skimage.feature import peak_local_max
from scipy.spatial import kdtree

from skimage.draw import line_aa

from .key_constants_pool import *
from .utils import polygon_iou
from .grasp_repr import GraspInLine


def draw_rect_on_img_with_corners(img, corners):
    """
    通过矩形框的4个corners的坐标确定一个抓取矩形框
    :param corners:
    :return:
    """
    y0, x0 = corners[0]
    y1, x1 = corners[1]
    y2, x2 = corners[2]
    y3, x3 = corners[3]
    line_r0, line_c0, _ = line_aa(int(y0), int(x0), int(y1), int(x1))
    line_r1, line_c1, _ = line_aa(int(y1), int(x1), int(y2), int(x2))
    line_r2, line_c2, _ = line_aa(int(y2), int(x2), int(y3), int(x3))
    line_r3, line_c3, _ = line_aa(int(y3), int(x3), int(y0), int(x0))

    img[line_r0, line_c0] = np.array([0, 255, 0])
    img[line_r2, line_c2] = np.array([0, 255, 0])
    img[line_r1, line_c1] = np.array([0, 0, 255])
    img[line_r3, line_c3] = np.array([0, 0, 255])

    return img


def evaluate_prediction(pred, info, n_cls, mode, angle_th=30, iou_th=0.25, classification=True, vis=False):
    """
    评估网络输出的结果；输入的pred和info都用batchsize=1
    pred中的结果与info中任意label满足以下两个条件，则认为预测正确：
        1、角度偏差小于30度
        2、两者IoU > 0.25
    :param pred: 输出结果
    :param info: 目标值
    :param mode: 计算graspness时使用的模式 —— peak, all, max
    :param n_cls: 角度分类个数
    :param angle_th: 角度阈值
    :param iou_th: iou阈值
    :return: 只要pred中存在一个满足上述条件的grasp，则返回True，否则返回False
    """
    len_interval = np.pi / n_cls
    angles_table = {0: 0.0}  # 存储每个类别对应的角度
    for i in range(n_cls):
        start_angle = i * len_interval
        end_angle = (i + 1) * len_interval
        mid = (start_angle + end_angle) / 2
        angles_table[i + 1] = mid

    graspness_pred = pred[KeyGraspnessPred]  # (h,w)
    # angle用softmax做最后的激活，已经被softmax和argmax处理过了
    grasp_angle_pred = pred[KeyGraspAnglePred]  # (h,w)
    grasp_width_pred = pred[KeyGraspWidthPred] * 200  # (h,w), 从[0,1]的范围恢复

    graspness_target = info[KeyGraspness].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_angle_target = info[KeyGraspAngle].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_angle_target_raw = info[KeyGraspAngleRaw].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_width_target = info[KeyGraspWidth].squeeze().cpu() * 200  # (1,h,w) -> (h,w)

    # 取得抓取概率高的抓取点，得到它们的二维坐标
    threshold_abs = 0.65
    if mode == 'peak':
        # 取所有的局部最大值对应的坐标位置
        min_distance = 10
        pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
        print('first query found ', pred_pts.shape[0])
        while pred_pts.shape[0] > 50:  # 如果局部最大值太多，则在局部中再取局部
            threshold_abs += 0.05
            pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
            if threshold_abs >= 0.95:
                break
        while pred_pts.shape[0] > 50:
            min_distance += 2
            pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
            if min_distance >= 30:
                break
        # print('threshold_abs={}, min_distance={}, 极大值数量={}'.format(threshold_abs, min_distance, pred_pts.shape[0]))
    elif mode == 'all':
        # 所有超过阈值的点都要
        r, c = np.where(graspness_pred >= threshold_abs)
        pred_pts = np.vstack([r, c]).T
        # 只取top-50进行验证
        idx = np.argsort(graspness_pred[r, c])
        pred_pts = pred_pts[idx[::-1]]
        pred_pts = pred_pts[:50]

    elif mode == 'max':
        # 只要预测值最大的那个位置
        loc = np.argmax(graspness_pred)
        row = loc // graspness_pred.shape[0]
        col = loc % graspness_pred.shape[0]
        pred_pts = np.array([[row, col]])
    else:
        raise ValueError("Not supported value for mode, available ('peak', 'all', 'max')")

    RADIUS = 30  # in pixel
    graspness_target_y, graspness_target_x = np.where(graspness_target == 1)
    graspness_target_pts = np.vstack([graspness_target_y, graspness_target_x]).T  # (n,2)
    query_tree = kdtree.KDTree(data=graspness_target_pts)
    print(f'Found {pred_pts.shape[0]} grasps, shape = {pred_pts.shape}')
    has_positive = False
    for idx in range(pred_pts.shape[0]):
        y_pred, x_pred = pred_pts[idx]
        angle_pred_cls = grasp_angle_pred[y_pred, x_pred]  # 预测抓取角度类别
        width_pred = grasp_width_pred[y_pred, x_pred]  # 预测宽度
        if classification:
            angle_pred = angles_table[angle_pred_cls]  # 分类要去找对应的角度值
        else:
            angle_pred = angle_pred_cls  # 由回归得到，输出就直接是角度值
            angle_pred = np.clip(angle_pred.cpu().numpy(), 0, np.pi)  # 直接限制在[0,pi]内
        # 将上述三个参数转换成一个抓取矩形框的表示形式
        grasp_rect_repr = GraspInLine.to_rect_repr(x_pred, y_pred, angle_pred, width_pred)
        if vis:
            info[KeyOriginalImageDebug4Pred] = draw_rect_on_img_with_corners(info[KeyOriginalImageDebug4Pred], grasp_rect_repr)  # 预测的矩形框结果画在图片上
        # 找到当前预测点附近RADIUS范围内的所有真实抓取点
        neighbor_indices = query_tree.query_ball_point((y_pred, x_pred), r=RADIUS, workers=8)  # List[int]
        # 逐个点检查是否预测正确
        for neighbor_idx in neighbor_indices:
            # 计算角度之差
            neighbor_p_y, neighbor_p_x = graspness_target_pts[neighbor_idx]
            angle_target = grasp_angle_target_raw[neighbor_p_y, neighbor_p_x]  # in rad
            width_target = grasp_width_target[neighbor_p_y, neighbor_p_x]  # in pixel
            raw_angle_diff = abs(angle_target - angle_pred)
            angle_diff = min(raw_angle_diff, np.pi - raw_angle_diff)  # 0和pi的位置是对称的
            if np.rad2deg(angle_diff) > angle_th:
                continue
            # 计算IoU
            target_rect = GraspInLine.to_rect_repr(neighbor_p_x, neighbor_p_y, angle_target, width_target)
            iou = polygon_iou(grasp_rect_repr, target_rect)
            # print(f'iou = {iou}')
            if iou >= iou_th:
                # has_positive = True
                # 记录这个pred grasp和gt grasp的角度和宽度差值
                return True

    return False
    # return has_positive


def evaluate_prediction_with_diff_calculated(pred, info, n_cls, mode, angle_th=30, iou_th=0.25, classification=True, vis=False):
    """
    评估网络输出的结果；输入的pred和info都用batchsize=1
    pred中的结果与info中任意label满足以下两个条件，则认为预测正确：
        1、角度偏差小于30度
        2、两者IoU > 0.25
    :param pred: 输出结果
    :param info: 目标值
    :param mode: 计算graspness时使用的模式 —— peak, all, max
    :param n_cls: 角度分类个数
    :param angle_th: 角度阈值
    :param iou_th: iou阈值
    :return: 只要pred中存在一个满足上述条件的grasp，则返回True，否则返回False
    """
    len_interval = np.pi / n_cls
    angles_table = {0: 0.0}  # 存储每个类别对应的角度
    for i in range(n_cls):
        start_angle = i * len_interval
        end_angle = (i + 1) * len_interval
        mid = (start_angle + end_angle) / 2
        angles_table[i + 1] = mid

    graspness_pred = pred[KeyGraspnessPred]  # (h,w)
    # angle用softmax做最后的激活，已经被softmax和argmax处理过了
    grasp_angle_pred = pred[KeyGraspAnglePred]  # (h,w)
    grasp_width_pred = pred[KeyGraspWidthPred] * 200  # (h,w), 从[0,1]的范围恢复

    graspness_target = info[KeyGraspness].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_angle_target = info[KeyGraspAngle].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_angle_target_raw = info[KeyGraspAngleRaw].squeeze().cpu().numpy()  # (1,h,w) -> (h,w)
    grasp_width_target = info[KeyGraspWidth].squeeze().cpu() * 200  # (1,h,w) -> (h,w)

    # 取得抓取概率高的抓取点，得到它们的二维坐标
    threshold_abs = 0.65
    if mode == 'peak':
        # 取所有的局部最大值对应的坐标位置
        min_distance = 10
        pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
        print('first query found ', pred_pts.shape[0])
        while pred_pts.shape[0] > 50:  # 如果局部最大值太多，则在局部中再取局部
            threshold_abs += 0.05
            pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
            if threshold_abs >= 0.95:
                break
        while pred_pts.shape[0] > 50:
            min_distance += 2
            pred_pts = peak_local_max(graspness_pred, min_distance=min_distance, threshold_abs=threshold_abs)
            if min_distance >= 30:
                break
        # print('threshold_abs={}, min_distance={}, 极大值数量={}'.format(threshold_abs, min_distance, pred_pts.shape[0]))
    elif mode == 'all':
        # 所有超过阈值的点都要
        r, c = np.where(graspness_pred >= threshold_abs)
        pred_pts = np.vstack([r, c]).T
        # 只取top-50进行验证
        idx = np.argsort(graspness_pred[r, c])
        pred_pts = pred_pts[idx[::-1]]
        pred_pts = pred_pts[:10]

    elif mode == 'max':
        # 只要预测值最大的那个位置
        loc = np.argmax(graspness_pred)
        row = loc // graspness_pred.shape[0]
        col = loc % graspness_pred.shape[0]
        pred_pts = np.array([[row, col]])
    else:
        raise ValueError("Not supported value for mode, available ('peak', 'all', 'max')")

    RADIUS = 30  # in pixel
    graspness_target_y, graspness_target_x = np.where(graspness_target == 1)
    graspness_target_pts = np.vstack([graspness_target_y, graspness_target_x]).T  # (n,2)
    query_tree = kdtree.KDTree(data=graspness_target_pts)
    print(f'Found {pred_pts.shape[0]} grasps, shape = {pred_pts.shape}')

    angle_diffs = []
    width_diffs = []

    for idx in range(pred_pts.shape[0]):
        y_pred, x_pred = pred_pts[idx]
        angle_pred_cls = grasp_angle_pred[y_pred, x_pred]  # 预测抓取角度类别
        width_pred = grasp_width_pred[y_pred, x_pred]  # 预测宽度
        if classification:
            angle_pred = angles_table[angle_pred_cls]  # 分类要去找对应的角度值
        else:
            angle_pred = angle_pred_cls  # 由回归得到，输出就直接是角度值
            angle_pred = np.clip(angle_pred.cpu().numpy(), 0, np.pi)  # 直接限制在[0,pi]内
        # 将上述三个参数转换成一个抓取矩形框的表示形式
        grasp_rect_repr = GraspInLine.to_rect_repr(x_pred, y_pred, angle_pred, width_pred)
        if vis:
            info[KeyOriginalImageDebug4Pred] = draw_rect_on_img_with_corners(info[KeyOriginalImageDebug4Pred], grasp_rect_repr)  # 预测的矩形框结果画在图片上
        # 找到当前预测点附近RADIUS范围内的所有真实抓取点
        neighbor_indices = query_tree.query_ball_point((y_pred, x_pred), r=RADIUS, workers=12)  # List[int]
        # 逐个点检查是否预测正确
        # print('len(neighbor_indices) = ', len(neighbor_indices))
        for neighbor_idx in neighbor_indices:
            # 计算角度之差
            neighbor_p_y, neighbor_p_x = graspness_target_pts[neighbor_idx]
            angle_target = grasp_angle_target_raw[neighbor_p_y, neighbor_p_x]  # in rad
            width_target = grasp_width_target[neighbor_p_y, neighbor_p_x]  # in pixel
            raw_angle_diff = abs(angle_target - angle_pred)
            angle_diff = min(raw_angle_diff, np.pi - raw_angle_diff)  # 0和pi的位置是对称的
            if np.rad2deg(angle_diff) > angle_th:
                continue
            # 计算IoU
            target_rect = GraspInLine.to_rect_repr(neighbor_p_x, neighbor_p_y, angle_target, width_target)
            iou = polygon_iou(grasp_rect_repr, target_rect)
            # print(f'iou = {iou}')
            if iou >= iou_th:
                # 记录这个pred grasp和gt grasp的角度和宽度差值
                angle_diffs.append(np.rad2deg(angle_diff))
                width_diffs.append(abs(float(width_target) - float(width_pred)))

    return angle_diffs, width_diffs
