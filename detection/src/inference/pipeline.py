# coding=utf-8
"""
高度封装的检测类,只需要给定rgb或者depth信息,就返回检测到的抓取结果
"""

import time

import pytransform3d.rotations as pyrot
import matplotlib.pyplot as plt
import numpy as np
import torch
import cv2
from scipy.spatial import KDTree

from .network import MixedPSPNet, PSPNet
from .infer_tools import pack_rgbd_image, pack_single_image, decode_graspmap, decode_net_output
from .key_constants_pool import *
from .utils import *
from .grasp_repr import *
from inference_3d.utils import create_point_cloud_from_depth_image, estimate_normals, rotmat_from_direction_angle


class CameraParams:
    """
    相机的内外参参数类
    """

    def __init__(self, cx, cy, fx, fy, fov, w=640, h=480, scale=1000.0):
        self._cx = cx
        self._cy = cy
        self._fx = fx
        self._fy = fy
        self._fov = fov
        self._depth_scale = scale
        self._w = w
        self._h = h

    @property
    def width(self):
        return self._w

    @property
    def height(self):
        return self._h

    @property
    def cx(self):
        return self._cx

    @property
    def cy(self):
        return self._cy

    @property
    def fx(self):
        return self._fx

    @property
    def fy(self):
        return self._fy

    @property
    def fov(self):
        return self._fov

    @property
    def scale(self):
        return self._depth_scale

    def to_matrix(self):
        return np.array([[self._fx, 0., self._cx],
                         [0., self._fy, self._cy],
                         [0., 0., 1.]])


def detection_log(msg, header='node_detection::pipeline', level='info'):
    color = 32
    if level == 'info':
        color = 32
    elif level == 'warn':
        color = 33
    elif level == 'err':
        color = 31
    else:
        color = 34
    print(f"\033[{color}m[{time.time()}] {header} => {msg}\033[0m")


class PlanarGraspDetector:
    """
    检测类,负责二维平面抓取的检测
    """

    KeyDetected = 'detected'
    KeyGraspCenter = 'grasp_center'
    KeyAngle = 'grasp_angle'
    KeyPreGraspCenter = 'pre_grasp_center'
    KeyRotMat = 'rotmat'
    KeyRotMatCam = 'rotmatcam'
    KeyPhysicalWidth = 'pwidth'
    KeyImgOriginalWithP = 'img_ori_with_p'
    KeyImgWithGrasp = 'img_with_grasp'
    KeyImgGraspness = 'img_with_graspness'
    KeyImgCropped = 'img_cropped'

    def __init__(self, model_path: str, camera_params: CameraParams,
                 use_rgb=True,
                 n_cls=80,
                 map_size=384,
                 normal_as_direction=False,
                 threshold=0.60,
                 use_rgbd=False) -> None:
        # 以当前文件夹为前缀
        self.model_path = os.path.join(os.path.dirname(__file__), model_path)
        self.use_rgb = use_rgb
        self.camera_params = camera_params
        self.map_size = map_size
        self.use_rgbd = use_rgbd

        self.n_cls = n_cls
        len_interval = np.pi / n_cls
        self.angles_table = {0: 0.0}  # 存储每个类别对应的角度
        self.device = torch.device('cuda')
        for i in range(n_cls):
            start_angle = i * len_interval
            end_angle = (i + 1) * len_interval
            mid = (start_angle + end_angle) / 2
            self.angles_table[i + 1] = mid

        detection_log(f'Loading model from {self.model_path}...')
        if not use_rgbd:
            in_channels = use_rgb * 3 + (not use_rgb) * 1
            # 单模态模型
            self.network = PSPNet(
                n_angle_cls=80, in_channels=in_channels, pretrained=False)
            self.network.load_state_dict(torch.load(
                model_path, map_location=self.device))
        else:
            # 双模态模型
            self.network = MixedPSPNet(n_angle_cls=80, pretrained=False)
            checkpoint = torch.load(self.model_path, map_location=self.device)
            self.network.load_state_dict(checkpoint[KeyStateDictNetParams])
        # self.network.load_state_dict(torch.load(model_path, map_location=self.device))
        detection_log(f'Done loading model from {self.model_path}')
        self.network.to(self.device)
        self.network.eval()
        self.left = 320 - self.map_size // 2  # (640 // 2 - self.map_size // 2)
        self.top = 240 - self.map_size // 2  # (480 // 2 - self.map_size // 2)
        self.normal_as_direction = normal_as_direction
        self.threshold = threshold
        self.width_scale = 200

    def _through_network(self, img, depth):
        if not self.use_rgbd:
            if self.use_rgb:
                packed = pack_single_image(
                    img=img, depth=None, device=self.device)
            else:
                # 仅使用深度图
                packed = pack_single_image(
                    img=None, depth=depth, device=self.device)
        else:
            # 准备rgb-d的输入
            # img = exposure.adjust_gamma(img, gamma=0.65)
            packed = pack_rgbd_image(
                img=img, depth=depth, out_size=self.map_size, device=self.device, depth_norm_method='range')

        with torch.no_grad():
            pred = self.network(packed)

        pred = decode_net_output(pred)
        graspness = pred[KeyGraspnessPred]
        angle = pred[KeyGraspAnglePred]
        width = pred[KeyGraspWidthPred]
        # 用于可视化
        if self.use_rgb:
            cropped_img = pred[KeyOriginalImage]
        else:
            cropped_img = pred[KeyOriginalDepth]

        return pred, graspness, angle, width, cropped_img

    def detect(self, img: np.ndarray, depth: np.ndarray):
        """
        检测操作

        :param img: rgb图,shape=(h, w, 3)
        :param depth: 深度图,shape=(h, w), 单位m
        """
        pred, graspness, angle, width, cropped_img = self._through_network(
            img, depth)

        # 带有grasp结果的图片
        img_with_grasps, pred_grasp_pts = decode_graspmap(graspness, angle, width,
                                                          cropped_img, self.angles_table,
                                                          n_grasp=5, threshold=self.threshold,
                                                          mode='local', width_scale=self.width_scale)

        if pred_grasp_pts.shape[0] == 0:
            # 没有检测到结果
            return (False, )
        tcp_cam, angle, rot_mat_cam, physical_grasp_width, img_original_with_p, rotmat_wrt_camera = self.gen_grasp_pose(
            pred_grasp_pts, angle, width, img, depth)
        # 将graspness附在原图上
        graspness_on_img = self.attach_graspness_on_img(
            graspness=graspness, img=cropped_img)

        ret_map = {}
        ret_map[self.KeyGraspCenter] = tcp_cam
        ret_map[self.KeyAngle] = angle
        ret_map[self.KeyRotMatCam] = rot_mat_cam
        ret_map[self.KeyImgWithGrasp] = img_with_grasps
        ret_map[self.KeyPhysicalWidth] = physical_grasp_width
        ret_map[self.KeyImgOriginalWithP] = img_original_with_p
        ret_map[self.KeyImgGraspness] = graspness_on_img
        ret_map[self.KeyRotMat] = rotmat_wrt_camera
        ret_map[self.KeyImgCropped] = cropped_img

        # return True, tcp_cam, angle, rot_mat_cam, img_with_grasps, physical_grasp_width, img_original_with_p, graspness_on_img, rotmat_wrt_camera, cropped_img
        return True, ret_map


    def gen_grasp_pose(self, grasp_pts, angle_map, width_map, rgb_img, depth_img):
        """
        生成平面抓取姿态

        :param grasp_pts: 提取处的在二维图像上的抓取点, shape=(n, 2) -> [y, x]
        :param angle_map: 角度图,shape=(h, w)
        :param width_map: 宽度图,shape=(h, w)
        :param rgb_img: 原图RGB,shape=(H, W, 3)
        :param depth_img: 深度图,shape=(H, W)
        """
        # 1. 从中选出概率最高的grasp执行
        # OPTIM 可能要更加合理的grasp selection scheme
        # selected_grasp = grasp_pts[0]   # 第一个就是概率最高的点
        detection_log(f'grasp_pts={grasp_pts}, shape={grasp_pts.shape}')
        # 最后是选取z最小的那个点开始抓，因为这个点最靠近相机，也就是在最上方的，可能抓取的成功率要高一点
        n_grasp = len(grasp_pts)

        # 这里尝试过取z最小的那个点作为抓取点
        grasp_pts_raw_y = grasp_pts[:, 0] + self.top
        grasp_pts_raw_x = grasp_pts[:, 1] + self.left
        grasp_pts_z = depth_img[grasp_pts_raw_y,
                                grasp_pts_raw_x] / self.camera_params.scale  # (n,)
        selected_idx = np.argmin(grasp_pts_z)
        selected_idx = 0
        selected_grasp = grasp_pts[selected_idx]
        # x_cam = (grasp_pts[:, 1] - self.camera_params.cx) * z / self.camera_params.fx
        # y_cam = (grasp_pts[:, 0] - self.camera_params.cy) * z / self.camera_params.fy

        y, x = selected_grasp
        angle = self.angles_table[angle_map[y, x]]  # rad
        width = width_map[y, x] * self.width_scale   # pixel

        # 2. 将grasp转化到相机坐标系下
        # selected_grasp转回原图坐标系（h=480, w=640）
        y_raw, x_raw = self.top + y, self.left + x
        # rgb_img = cv2.circle(rgb_img, (x_raw, y_raw), radius=5, color=(255, 0, 0), thickness=cv2.FILLED)
        # 得到深度,单位化为m
        z = depth_img[y_raw, x_raw] / self.camera_params.scale
        # 抓取点转到相机坐标系下
        x_cam = (x_raw - self.camera_params.cx) * z / self.camera_params.fx
        y_cam = (y_raw - self.camera_params.cy) * z / self.camera_params.fy
        grasp_point_cam_3d = np.array([x_cam, y_cam, z])
        # detection_log(f'detection result on image: x_raw = {x_raw}, y_raw={y_raw}, z={z}')
        # detection_log(f'detection result on camera coordinate: x = {x_cam}, y={y_cam}, z={z}')

        grasps_orientation = self.cal_normal_at_point(self.camera_params,
                                                      depth=depth_img,
                                                      target=grasp_point_cam_3d,
                                                      topdown=not self.normal_as_direction)

        detection_log(
            f'grasps approach direction = {grasps_orientation}, width in pixel = {width}')
        gripper_frame_cam = GripperFrame.init(
            grasp_point_cam_3d, grasps_orientation, -angle - np.pi / 2)
        grasp_pose_cam = gripper_frame_cam.to_6dpose()

        # 夹爪在相机坐标系下的旋转矩阵
        rotation_mat_cam = grasp_pose_cam[:3, :3]   # (3,3)
        physical_grasp_width = width / self.map_size * 2 * z * \
            np.tan(np.deg2rad(self.camera_params.fov *
                   self.map_size / depth_img.shape[0] * 0.5))
        physical_grasp_width *= 1.22  # 稍微放宽一点
        # 最终实施抓取的夹爪张开宽度
        physical_grasp_width_final = np.clip(
            physical_grasp_width, 0.0, 0.080)  # 限制在最大抓取宽度内,单位m

        detection_log(
            f'angle = {angle} rad ({np.rad2deg(angle)} degrees), '
            'grasps_orientation = {grasps_orientation}, '
            'width = {physical_grasp_width_final}({physical_grasp_width})')

        rotmat_wrt_camera = rotmat_from_direction_angle(
            direction=-grasps_orientation.reshape((1, 3)), angle=angle-np.pi/2)

        return grasp_point_cam_3d, angle, rotation_mat_cam, physical_grasp_width_final, rgb_img, rotmat_wrt_camera

    def get_physical_grasp_width(self, depth_img, grasp_point_cam_3d, angle, width, z):
        grasps_orientation = self.cal_normal_at_point(self.camera_params,
                                                      depth=depth_img,
                                                      target=grasp_point_cam_3d,
                                                      topdown=not self.normal_as_direction)
        detection_log(
            f'grasps approach direction = {grasps_orientation}, width in pixel = {width}')
        gripper_frame_cam = GripperFrame.init(
            grasp_point_cam_3d, grasps_orientation, -angle - np.pi / 2)
        grasp_pose_cam = gripper_frame_cam.to_6dpose()

        # 夹爪在相机坐标系下的旋转矩阵
        rotation_mat_cam = grasp_pose_cam[:3, :3]   # (3,3)
        physical_grasp_width = width / self.map_size * 2 * z * \
            np.tan(np.deg2rad(self.camera_params.fov *
                              self.map_size / depth_img.shape[0] * 0.5))
        physical_grasp_width *= 1.1  # 稍微放宽一点
        # 最终实施抓取的夹爪张开宽度
        physical_grasp_width_final = np.clip(
            physical_grasp_width, 0.0, 0.080)  # 限制在最大抓取宽度内,单位m

        return physical_grasp_width_final

    def attach_graspness_on_img(self, graspness, img):
        """将graspness附加在img图片上

        Args:
            graspness (np.ndarray): shape(H, W)
            img (np.ndarray): shape (H, W, 3)
        """
        # 屏蔽小于阈值的点
        graspness[graspness < self.threshold] = 0.0
        cmap = 'magma'
        # cmap = 'Oranges'
        cmap_func = plt.get_cmap(cmap)
        heatmap_cmapped = cmap_func(graspness)
        heatmap_cmapped = np.delete(heatmap_cmapped, 3, 2)
        heatmaps_drawn = np.clip(
            heatmap_cmapped * 255, 0, 255).astype(np.uint8)
        alpha = 0.75
        mix = np.clip(
            (1 - alpha) * (img * 255).astype(np.uint8) + alpha * heatmaps_drawn,
            0, 255
        ).astype(np.uint8)

        return mix

    @staticmethod
    def cal_normal_at_point(camera_params, depth, target, topdown=True):
        """获取在target处的法向量

        Args:
            depth (np.ndarray): 深度图,用来生成点云
            target (np.ndarray): 目标点坐标
            topdown (bool): 是否进行top down grasping
        """
        if topdown:
            # top-down grasping, 在moveit里面,x轴朝下了
            return np.array([1., 0., 0.])
        # get normal vector at target point
        pointcloud = create_point_cloud_from_depth_image(
            depth=depth, camera=camera_params, organized=False)
        normals = estimate_normals(pointcloud)
        tree = KDTree(data=pointcloud, leafsize=20)
        neighbors_index = tree.query_ball_point(x=target, r=0.005, workers=-1)
        if len(neighbors_index) == 0:
            neighbors_index = tree.query_ball_point(
                x=target, r=0.01, workers=-1)
            if len(neighbors_index) == 0:
                detection_log(
                    f'No neighbors found, switch to top-down grasping.', level='warn')
                return np.array([1., 0., 0.])
        neighbors_normals = normals[neighbors_index]
        # average normals
        normals_avg = neighbors_normals.sum(axis=0)
        normals_avg /= np.linalg.norm(normals_avg)  # shape=(3,)
        if np.any(np.isnan(normals_avg)):
            detection_log(
                f'Normal of neighbors is nan, switch to top-down grasping.', level='warn')
            return np.array([1., 0., 0.])
        return normals_avg
