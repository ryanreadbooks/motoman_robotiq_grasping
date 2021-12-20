"""
高度封装的检测类，只需要给定rgb或者depth信息，就返回检测到的抓取结果
"""

import os
import time


import numpy as np
import torch
from scipy.spatial import KDTree

from network import PSPNet
from infer_tools import pack_single_image, decode_graspmap, decode_net_output
from key_constants_pool import *
from utils import *


class CameraParams:
    """
    相机的内外参参数类
    """

    def __init__(self, cx, cy, fx, fy, fov):
        self._cx = cx
        self._cy = cy
        self._fx = fx
        self._fy = fy
        self._fov = fov

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

    def to_matrix(self):
        return np.array([[self._fx, 0., self._cx], 
                         [0., self._fy, self._cy], 
                         [0., 0., 1.]])


class PlanarGraspDetector:
    """
    检测类，负责二维平面抓取的检测
    """

    def __init__(self, model_path: str, camera_params: CameraParams, use_rgb = True, n_cls=80, map_size=384) -> None:
        self.model_path = model_path
        self.use_rgb = use_rgb
        self.camera_params = camera_params
        self.map_size = map_size

        self.n_cls = n_cls
        len_interval = np.pi / n_cls
        self.angles_table = {0: 0.0}  # 存储每个类别对应的角度
        for i in range(n_cls):
            start_angle = i * len_interval
            end_angle = (i + 1) * len_interval
            mid = (start_angle + end_angle) / 2
            self.angles_table[i + 1] = mid

        in_channels = use_rgb * 3 + (not use_rgb) * 1
        self.network = PSPNet(n_angle_cls=80, in_channels=in_channels, pretrained=False)
        self.device = torch.device('cuda')
        self.network.load_state_dict(torch.load(model_path, map_location=self.device))
        self.network.to(self.device)
        self.network.eval()     

    def detect(self, img: np.ndarray, depth: np.ndarray):
        """
        检测操作

        :param img: rgb图，shape=(h, w, 3)
        :param depth: 深度图，shape=(h, w), 单位m
        """
        if self.use_rgb:
            packed = pack_single_image(img=img, depth=None, device=self.device)
        else:
            # 仅使用深度图
            packed = pack_single_image(img=None, depth=depth, device=self.device)

        with torch.no_grad():
            pred = self.network(packed)
        
        pred = decode_net_output(pred)
        graspness = pred[KeyGraspnessPred]
        angle = pred[KeyGraspAnglePred]
        width = pred[KeyGraspWidthPred]
        cropped_rgb = pred[KeyOriginalImage]

        # 带有grasp结果的图片
        img_with_grasps, pred_grasp_pts = decode_graspmap(graspness, angle, width, 
                                                        cropped_rgb, self.angles_table, 
                                                        n_grasp=5, mode='local')
        self.gen_grasp_pose(pred_grasp_pts, angle, width, img, depth)


    def gen_grasp_pose(self, grasp_pts, angle_map, width_map, rgb_img, depth_img):
        """
        生成平面抓取姿态

        :param grasp_pts: 提取处的在二维图像上的抓取点， shape=(n, 2) -> [y, x]
        :param angle_map: 角度图，shape=(h, w)
        :param widht_map: 宽度图，shape=(h, w)
        """
        # 1. 从中选出概率最高的grasp执行
        # TODO 可能要更加合理的grasp selection scheme
        selected_grasp = grasp_pts[0]   # 第一个就是概率最高的点
        y, x = selected_grasp
        angle = self.angles_table[angle_map[y, x]]  # rad
        width = width_map[y, x] * 200   # pixel
        
        # 2. 将grasp转化到相机坐标系下
        # selected_grasp转回原图坐标系（h=480, w=640）
        left = 320 - self.map_size // 2 # (640 // 2 - self.map_size // 2)
        top = 240 - self.map_size // 2 # (480 // 2 - self.map_size // 2)
        y_raw, x_raw = top + y, left + x
        # 得到深度
        z = depth_img[y_raw, x_raw]
        # 抓取点转到相机坐标系下
        x = (x_raw - self.camera_params.cx) * z / self.camera_params.fx
        y = (y_raw - self.camera_params.cy) * z / self.camera_params.fy
        grasp_point_cam_3d = np.array([x, y, z])
        # 末端姿态
        cloud = raw_generate_pc(rgb=rgb_img, depth=depth_img, cam_intrin=self.camera_params.to_matrix())
        cloud.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=0.005,
                max_nn=30
            )
        )
        cloud.normalize_normals()
        cloud.orient_normals_towards_camera_location()  # 法向量指向相机位置
        # 为抓取点找到点云中最近的点，并且取出法向量
        tree = KDTree(np.asarray(cloud.points), leafsize=16)
        distances, indices = tree.query(grasp_point_cam_3d, k=1, workers=4)
        cloud_normals = np.asarray(cloud.normals)
        grasps_orientation = cloud_normals[indices]    # (3,)

        # 3. 将相机坐标系下的grasp转换到机器人基座标系下，能够直接被moveit接收的格式
