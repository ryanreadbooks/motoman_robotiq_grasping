import os
import time

from typing import List

import numpy as np
from copy import deepcopy
import torch
from skimage.draw import line_aa
from scipy.spatial import KDTree

from .post_process import post_process_output
from .utils.camera_data import CameraData
from .utils.dataset_processing.grasp import *
from .models.grconvnet3 import GenerativeResnet

from inference_3d.pipeline import GraspNotFoundError
from inference.pipeline import CameraParams, detection_log
from inference.grasp_repr import *
from inference.pipeline import PlanarGraspDetector
from inference_3d.utils import create_point_cloud_from_depth_image, estimate_normals, rotmat_from_direction_angle


def grconv_log(msg, level='info'):
    detection_log(msg, header='grconv_detection', level=level)


class GraspGenerator:
    """
    GRConvNet的检测方法
    """

    KeyDetected = 'detected'
    KeyGraspCenter = 'grasp_center'
    KeyPreGraspCenter = 'pre_grasp_center'
    KeyRotMat = 'rotmat'
    KeyPhysicalWidth = 'pwidth'
    KeyImgOriginalWithP = 'img_ori_with_p'
    KeyImgWithGrasp = 'img_with_grasp'
    KeyImgGraspness = 'img_with_graspness'

    def __init__(self, camera_params: CameraParams, model_path: str='', normal_as_direction=False):
        self.model_path = model_path
        if self.model_path == '':
            self.model_path = os.path.join(os.path.dirname(__file__), 'ckpt', 'grconv3-v2.pth')
            grconv_log(f'Use default model state dict from {self.model_path}...')
        self.model = GenerativeResnet(dropout=True)
        state_dict = torch.load(self.model_path)
        self.model.load_state_dict(state_dict)
        self.device = torch.device('cuda')
        self.model.to(self.device)
        self.model.eval()
        
        self.map_size = 300
        self.cam_data = CameraData(output_size=self.map_size, include_depth=True, include_rgb=True)
        self.camera_params = camera_params
        self.normal_as_direction = normal_as_direction
        self.threshold = 0.5

    def generate(self, rgb, depth):
        # Get RGB-D image from camera
        depth = np.expand_dims(depth, axis=2)
        x, _, _, rbg_img_vis = self.cam_data.get_data(rgb=rgb, depth=depth)

        # Predict the grasp pose using the saved model
        with torch.no_grad():
            xc = x.to(self.device)
            pred = self.model.predict(xc)

        q_img, ang_img, width_img = post_process_output(
            pred['pos'], pred['cos'], pred['sin'], pred['width'])
        grasps: List[Grasp] = detect_grasps(q_img, ang_img, width_img, no_grasps=5)

        graspness_on_img = self.attach_graspness_on_img(q_img, rbg_img_vis)

        if len(grasps) == 0:
            raise GraspNotFoundError('Can not find any grasp configurations')
        grconv_log(f'Number of potential grasps = {len(grasps)}')
        chosen_grasp = grasps[0]

        # 得到相机坐标系下的抓取点坐标 单位m
        pos_z = depth[chosen_grasp.center[0] + self.cam_data.top_left[0],
                      chosen_grasp.center[1] + self.cam_data.top_left[1]] / self.camera_params._depth_scale
        pos_x = np.multiply(chosen_grasp.center[1] + self.cam_data.top_left[1] - self.camera_params.cx,
                            pos_z / self.camera_params.fx)
        pos_y = np.multiply(chosen_grasp.center[0] + self.cam_data.top_left[0] - self.camera_params.cy,
                            pos_z / self.camera_params.fy)

        if pos_z == 0:
            raise GraspNotFoundError('Can not form valid grasp configuration, because pos_z is 0.0')

        # 得到相机坐标系下的抓取点位置
        target = np.asarray([pos_x, pos_y, pos_z]).squeeze()
        # 得到抓取角度
        angle = chosen_grasp.angle
        width = chosen_grasp.width

        # 可视化操作检测结果
        img_with_grasps = self.vis_grasp(grasps=grasps, image=deepcopy(rbg_img_vis), n_grasp=5)

        grconv_log(f'target={target}, angle={angle}, width={width}')
        
        return target, angle, width, rbg_img_vis, img_with_grasps, graspness_on_img

    def detect(self, img: np.ndarray, depth: np.ndarray):
        """
        检测操作

        :param img: rgb图,shape=(h, w, 3)
        :param depth: 深度图,shape=(h, w), 单位m
        """
        # 得到相机坐标系下的抓取点坐标等信息
        target, angle, width, rbg_img_vis, img_with_grasps, _ = self.generate(img, depth)
        # 生成抓取姿态
        tcp, rotmat, phy_width = self.gen_grasp_pose(target, angle, width, depth)
        
        return tcp, rotmat, angle, phy_width, rbg_img_vis, img_with_grasps


    def gen_grasp_pose(self, grasp_target: np.ndarray, angle: float, width: float, depth: np.ndarray):
        # 抓取姿态计算 
        z = grasp_target[2]
        grasp_orientation = PlanarGraspDetector.cal_normal_at_point(self.camera_params, 
                                                                    depth=depth,
                                                                    target=grasp_target,
                                                                    topdown=not self.normal_as_direction)
        # grasp_orientation = np.array([1., 0., 0.])
        gripper_frame_cam = GripperFrame.init(grasp_target, grasp_orientation, angle)
        grasp_pose_cam = gripper_frame_cam.to_6dpose()
        # 夹爪在相机坐标系下的旋转矩阵
        rotation_mat_cam = grasp_pose_cam[:3, :3]   # (3,3)

        # 抓取宽度计算
        physical_grasp_width = width / self.map_size * 2 * z * np.tan(np.deg2rad(self.camera_params.fov * self.map_size / 480 * 0.5))
        physical_grasp_width *= 1.1
        physical_grasp_width_final = np.clip(physical_grasp_width, 0.0, 0.120) # 限制在最大抓取宽度内,单位m

        tcp_position_cam = grasp_target + rotation_mat_cam[:, -1] * 0.003

        return tcp_position_cam, rotation_mat_cam, physical_grasp_width_final


    def vis_grasp(self, grasps: List[Grasp], image: np.ndarray, n_grasp=5):
        """可视化grasp到图片上

        Args:
            grasps (List[Grasp]): 所有的grasp
            image (np.ndarray): 需要在这个图片上画上grasp
            n_grasp (int, optional): 从grasps中选多少个grasp画在image上. Defaults to 5.
        """
        for i in range(min(len(grasps), n_grasp)):
            grasp_rect: GraspRectangle = grasps[i].as_gr
            corners = grasp_rect.points  # 4个角点
            # 可视化4个角点
            y0, x0 = corners[0]
            y1, x1 = corners[1]
            y2, x2 = corners[2]
            y3, x3 = corners[3]
            line_r0, line_c0, _ = line_aa(int(y0), int(x0), int(y1), int(x1))
            line_r1, line_c1, _ = line_aa(int(y1), int(x1), int(y2), int(x2))
            line_r2, line_c2, _ = line_aa(int(y2), int(x2), int(y3), int(x3))
            line_r3, line_c3, _ = line_aa(int(y3), int(x3), int(y0), int(x0))
            image[np.clip(line_r0, 0, self.map_size - 1), np.clip(line_c0, 0, self.map_size - 1)] = np.array([0, 255, 0])
            image[np.clip(line_r2, 0, self.map_size - 1), np.clip(line_c2, 0, self.map_size - 1)] = np.array([0, 255, 0])
            image[np.clip(line_r1, 0, self.map_size - 1), np.clip(line_c1, 0, self.map_size - 1)] = np.array([0, 0, 255])
            image[np.clip(line_r3, 0, self.map_size - 1), np.clip(line_c3, 0, self.map_size - 1)] = np.array([0, 0, 255])

        return image


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
