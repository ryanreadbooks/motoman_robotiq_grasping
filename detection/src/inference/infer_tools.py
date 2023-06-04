"""
written by ryanreadbooks
date: 2021/11/27
"""
# coding=utf-8
from typing import Dict
import torch
from skimage.filters import gaussian
from skimage.feature import peak_local_max
import numpy as np
import cv2
import copy

from .key_constants_pool import *
from .grasp_repr import GraspInLine


def normalize_min_max(data):
    """min-max归一化处理

    Args:
        data (np.ndarray): 需要归一化的数据

    Returns:
        np.ndarray: 归一化的数据
    """
    return (data - data.min()) / (data.max() - data.min())


def normalize_range(data, lower=-1, upper=1):
    """均值归一化

    Args:
        data (np.ndarray): 需要归一化的数据
        lower (int, optional): 归一化后的下界. Defaults to -1.
        upper (int, optional): 归一化后的上界. Defaults to 1.

    Returns:
        _type_: 归一化的数据
    """
    return np.clip(data - data.mean(), lower, upper)


def process_rgb(img: np.ndarray, out_size, device):
    """处理rgb图片

    Args:
        img (np.ndarray): (h,w,3) rgb图
        out_size (Tuple): _description_
        device (_type_): _description_

    Returns:
        torch.Tensor, np.ndarray: 给网络输入的rgb和用来可视化的rgb
    """
    h, w = img.shape[:2]
    left = w // 2 - out_size // 2
    top = h // 2 - out_size // 2
    img_crop = img[top: top + out_size, left: left + out_size]
    img = img_crop.astype(np.float32) / 255.0
    img -= img.mean()
    img = img.transpose((2, 0, 1))  # 转成[c,h,w]格式
    # 变成tensor
    return torch.from_numpy(img)[None].to(device), img_crop  # (1,c,h,w)


def process_depth(depth, out_size, device, norm_method='range'):
    """处理深度图

    Args:
        depth (np.ndarray): (h,w) 深度图
        out_size (_type_): _description_
        device (_type_): _description_

    Returns:
        torch.Tensor, np.ndarray: 给网络输入的depth和用来可视化的depth
    """
    h, w = depth.shape
    left = w // 2 - out_size // 2
    top = h // 2 - out_size // 2
    depth = depth[top: top + out_size, left:left + out_size]
    if norm_method == 'min_max':
        depth = normalize_min_max(depth)
    else:
        depth = np.clip(depth - depth.mean(), -1, 1)        
    depth_tensor = torch.from_numpy(depth)[None, None].to(device)  # (1,1,h,w)

    return depth_tensor, depth


def pack_single_image(img: np.ndarray = None, depth: np.ndarray = None, out_size=384, device=torch.device('cuda')):
    """
    处理单张图片,处理成能够直接输入到网络的tensor,包括归一化预处理
    :param img: (h,w,3), np.ndarray
    :param depth: (h,w), np.ndarray,单位m
    :param out_size: 输出尺寸,默认截取输入图片中间的out_size部分区域
    :return: ret : dict with only KeyNetInput item
    """
    if img is None and depth is None:
        raise ValueError('img and depth can not be both None')
    if img is not None and depth is not None:
        raise ValueError('either img or depth is None. `Use pack_rgbd_image` instead')
    if img is not None:
        img_tensor, img_crop = process_rgb(img=img, out_size=out_size, device=device)
        return {KeyNetInput: img_tensor, KeyOriginalImage: img_crop}
    else:
        depth_tensor, depth = process_depth(depth=depth, out_size=out_size, device=device, norm_method='range')
        return {KeyNetInput: depth_tensor, KeyOriginalDepth: depth}


def pack_rgbd_image(img: np.ndarray, depth: np.ndarray, out_size=384, device=torch.device('cuda'), depth_norm_method='min_max'):
    """
    处理输入的rgb和depth图片,包括归一化处理

    Args:
        img (np.ndarray): (h,w,3), np.ndarray
        depth (np.ndarray): (h,w), np.ndarray
        out_size (int, optional): 输出尺寸, 默认截取输入图片中间的out_size的部分区域. Defaults to 384.
        device (_type_, optional): 设备. Defaults to torch.device('cuda').
        depth_norm_method (str, optional): 归一化深度图的方式. (min_max或者range), 默认min_max
    """
    if img is None and depth is None:
        raise ValueError('img and depth can not be None')
    rgb_tensor, cropped_rgb = process_rgb(img=img, out_size=out_size, device=device)
    depth_tensor, cropped_depth = process_depth(depth=depth, out_size=out_size, device=device, norm_method=depth_norm_method)

    return {KeyColorImg: rgb_tensor.float(), KeyDepthImg: depth_tensor.float(), KeyOriginalImage: cropped_rgb, KeyOriginalDepth: cropped_depth}


def decode_net_output(pred: Dict, classification=True):
    """
    对网络的预测结果进行后处理
    :param pred: 包含了三个输出结果的字典 batchsize=1,内含在device上的torch.Tensor
    :param classification:
    :return: pred,新增进解码结果
    """
    graspness = pred[KeyGraspnessOut]  # (1,1,h,w)
    angle = pred[KeyGraspAngleOut]  # (1,1+n_cls,h,w)
    width = pred[KeyGraspWidthOut]  # (1,1,h,w)

    graspness = gaussian(graspness.squeeze().sigmoid().cpu().numpy(), sigma=2.0, preserve_range=True)  # (h,w)
    # graspness = graspness.squeeze().sigmoid().cpu().numpy()  # (h,w)
    width = width.squeeze().cpu().sigmoid().numpy()  # (h,w), 这里的width是归一化了的
    if classification:
        angle = torch.softmax(angle, dim=1)  # (1,1+n_cls,h,w)
        angle = torch.argmax(angle, dim=1).squeeze().cpu().numpy()  # (h,w)
    else:
        # 输出的是用regression得到的连续角度值
        # angle (1,1,h,w)
        angle = angle.squeeze()  # (h,w)

    width = gaussian(width, sigma=1.0, preserve_range=True)

    pred[KeyGraspnessPred] = graspness
    pred[KeyGraspAnglePred] = angle
    pred[KeyGraspWidthPred] = width

    return pred


def decode_graspmap(graspness, angle, width, original_img, angles_table, n_grasp=5, threshold=0.55, mode='global', width_scale=200):
    """
    解析grasp map,得到一系列的GraspInLine
    :param graspness: (h,w)
    :param angle: (h,w)
    :param width: (h,w)
    :param original_img: (h,w,3)
    :param n_grasp: 要从grasp map中选出多少个grasp的最大值
    :param threshold: 概率的阈值
    :param n_cls: 角度分类的数目
    :param mode: all 或者 peak,all从全局中选取n个大于threshold的grasp point;peak选取局部最大的grasp
    :param width_scale: 宽度的scale系数, 默认=200
    :return:
    """
    assert mode in ('global', 'local'), "mode should be either 'global' or 'local', {:s} not supported.".format(mode)
    h, w = graspness.shape

    if mode == 'global':
        # 所有超过阈值的点都要
        r, c = np.where(graspness >= threshold)
        pred_pts = np.vstack([r, c]).T
        # 只取top-n_grasp进行验证
        idx = np.argsort(graspness[r, c])
        pred_pts = pred_pts[idx[::-1]]
        pred_pts = pred_pts[:n_grasp]  # 只取前n个作为输出
    else:
        # 多个物体的时候,局部最大值的都取
        pred_pts = peak_local_max(image=graspness, min_distance=2, threshold_abs=threshold, num_peaks=n_grasp)
    original_img_for_drawing = copy.deepcopy(original_img)
    if len(original_img.shape) == 2:
        # 输入的original_img是深度图
        original_img_for_drawing = cv2.applyColorMap((original_img_for_drawing * 255).astype(np.uint8), cv2.COLORMAP_JET)
    for (r, c) in pred_pts:
        angle_value = angles_table[angle[r, c]]  # rad
        width_value = width[r, c] * width_scale  # pixel
        grasp = GraspInLine(x=c, y=r, width=width_value, angle=angle_value, shape=(h, w), corners=None)
        original_img_for_drawing = grasp.draw_on_img(original_img_for_drawing)

    return original_img_for_drawing, pred_pts
