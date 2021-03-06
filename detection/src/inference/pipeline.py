# coding=utf-8
"""
高度封装的检测类,只需要给定rgb或者depth信息,就返回检测到的抓取结果
"""

import os
import time

import numpy as np
import torch
import cv2
# from scipy.spatial import KDTree

from .network import PSPNet
from .infer_tools import pack_single_image, decode_graspmap, decode_net_output
from .key_constants_pool import *
from .utils import *
from .grasp_repr import *


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


def detection_log(msg):
    print(f"node_detection::pipeline => {msg}")


class PlanarGraspDetector:
    """
    检测类,负责二维平面抓取的检测
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

        :param img: rgb图,shape=(h, w, 3)
        :param depth: 深度图,shape=(h, w), 单位m
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
        
        if pred_grasp_pts.shape[0] == 0:
            # 没有检测到结果
            return (False, )
        tcp_cam, angle, rot_mat_cam, physical_grasp_width, img_original_with_p = self.gen_grasp_pose(pred_grasp_pts, angle, width, img, depth)

        return True, tcp_cam, angle, rot_mat_cam, img_with_grasps, physical_grasp_width, img_original_with_p


    def gen_grasp_pose(self, grasp_pts, angle_map, width_map, rgb_img, depth_img):
        """
        生成平面抓取姿态

        :param grasp_pts: 提取处的在二维图像上的抓取点, shape=(n, 2) -> [y, x]
        :param angle_map: 角度图,shape=(h, w)
        :param widht_map: 宽度图,shape=(h, w)
        :param rgb_img: 原图RGB,shape=(H, W, 3)
        :param depth_img: 深度图,shape=(H, W)
        """
        # 1. 从中选出概率最高的grasp执行
        # TODO 可能要更加合理的grasp selection scheme
        selected_grasp = grasp_pts[0]   # 第一个就是概率最高的点
        y, x = selected_grasp
        angle = self.angles_table[angle_map[y, x]]  # rad
        width = width_map[y, x] * 150   # pixel
        
        # 2. 将grasp转化到相机坐标系下
        # selected_grasp转回原图坐标系（h=480, w=640）
        left = 320 - self.map_size // 2 # (640 // 2 - self.map_size // 2)
        top = 240 - self.map_size // 2 # (480 // 2 - self.map_size // 2)
        y_raw, x_raw = top + y, left + x
        rgb_img = cv2.circle(rgb_img, (x_raw, y_raw), radius=5, color=(255, 0, 0), thickness=cv2.FILLED)
        # 得到深度,单位化为m
        z = depth_img[y_raw, x_raw] / 1000.0
        # 抓取点转到相机坐标系下
        x_cam = (x_raw - self.camera_params.cx) * z / self.camera_params.fx
        y_cam = (y_raw - self.camera_params.cy) * z / self.camera_params.fy
        grasp_point_cam_3d = np.array([x_cam, y_cam, z])
        detection_log(f'detection result on image: x_raw = {x_raw}, y_raw={y_raw}, z={z}')
        detection_log(f'detection result on camera coordinate: x = {x_cam}, y={y_cam}, z={z}')
        # TODO 末端姿态按照抓取点的法向量来,现在暂时不用法向量
        # 这里depth_scale给1,因为外面已经scale过了depth了
        # cloud = raw_generate_pc(rgb=rgb_img, depth=depth_img, depth_scale=1000.0, cam_intrin=self.camera_params.to_matrix())
        # if cloud.is_empty():
        #     raise RuntimeError('the input point cloud is empty, rgb shape={:s}, depth shape={:s}'.format(str(rgb_img.shape), str(depth_img.shape)))
        # cloud.estimate_normals(
        #     search_param=o3d.geometry.KDTreeSearchParamHybrid(
        #         radius=0.005,
        #         max_nn=30
        #     )
        # )
        # cloud.normalize_normals()
        # cloud.orient_normals_towards_camera_location()  # 法向量指向相机位置

        # 为抓取点找到点云中最近的点,并且取出法向量
        # tree = KDTree(np.asarray(cloud.points), leafsize=16)
        # distances, indices = tree.query(grasp_point_cam_3d, k=1, workers=4)
        # cloud_normals = np.asarray(cloud.normals)
        # grasps_orientation = -cloud_normals[indices]    # (3,)

        # TODO 存在问题： top-down grasping, 在moveit里面,x轴朝下了
        grasps_orientation = np.array([1., 0., 0.])
        gripper_frame_cam = GripperFrame.init(grasp_point_cam_3d, grasps_orientation, -angle - np.pi / 2)
        grasp_pose_cam = gripper_frame_cam.to_6dpose()

        # 夹爪在相机坐标系下的旋转矩阵
        rotation_mat_cam = grasp_pose_cam[:3, :3]   # (3,3)
        physical_grasp_width = width / self.map_size * 2 * z * np.tan(np.deg2rad(self.camera_params.fov * self.map_size / depth_img.shape[0] * 0.5))
        physical_grasp_width *= 1.1 # 稍微放宽一点
        # 最终实施抓取的夹爪张开宽度
        physical_grasp_width_final = np.clip(physical_grasp_width, 0.0, 0.120) # 限制在最大抓取宽度内,单位m

        detection_log(f'angle = {angle} rad ({np.rad2deg(angle)} degrees), grasps_orientation = {grasps_orientation}, width = {physical_grasp_width_final}({physical_grasp_width})')
        detection_log(rotation_mat_cam)
        # 往前前进1cm,得到最终夹爪末端TCP应该到达的位置
        tcp_position_cam = grasp_point_cam_3d + rotation_mat_cam[:, -1] * 0.01
        
        return tcp_position_cam, angle, rotation_mat_cam, physical_grasp_width_final, rgb_img
        