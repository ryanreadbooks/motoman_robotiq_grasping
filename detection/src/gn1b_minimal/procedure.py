"""
written by ryanreadbooks
brief: 
date: 2022/5/19
"""
import os
import time
import numpy as np
import copy
import open3d as o3d

import torch
from graspnetAPI import GraspGroup

from .libs.model.graspnet import GraspNet, pred_decode
from .libs.collision_detector import ModelFreeCollisionDetector
from .libs.data_utils import CameraInfo, create_point_cloud_from_depth_image
from inference_3d.pipeline import GraspNotFoundError


def track_log(msg, level='info'):
    color = 32
    if level == 'info':
        color = 32
    elif level == 'warn':
        color = 33
    elif level == 'err':
        color = 31
    else:
        color = 34
    print(f"\033[{color}m[{time.time()}] GN1B-Minimal => {msg}\033[0m")


########################### 一些配置参数 ###########################
dirname = os.path.dirname(__file__)
ckpt_path = os.path.join(dirname, 'ckpt', 'checkpoint-rs.tar')
num_point = 20000
num_view = 300
collision_thresh = 0.01
voxel_size = 0.01
# 工作区mask
workspace_mask = np.zeros((480, 640), dtype=np.uint8)
# workspace范围 h,w = 288, 384
workspace_mask[96: 384, 128: 512] = 1
workspace_mask = np.where(workspace_mask == 1, True, False)


def make_prediction(color, depth, camera):
    """
    给定彩色图片，深度图片，内参矩阵
    :param color: 彩色图片
    :param depth: 深度图片
    :param intrinsic: 内参矩阵
    :param factor_depth: 深度图的scale
    :return:
    """

    # 创建点云
    cloud = create_point_cloud_from_depth_image(depth, camera, organized=True)

    # get valid points
    mask = (workspace_mask & (depth > 0))
    cloud_masked = cloud[mask]
    color_masked = color[mask]
    # sample points
    if len(cloud_masked) >= num_point:
        idxs = np.random.choice(len(cloud_masked), num_point, replace=False)
    else:
        idxs1 = np.arange(len(cloud_masked))
        idxs2 = np.random.choice(len(cloud_masked), num_point - len(cloud_masked), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    cloud_sampled = cloud_masked[idxs]
    color_sampled = color_masked[idxs]

    # convert data
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(cloud_masked.astype(np.float32))
    cloud.colors = o3d.utility.Vector3dVector(color_masked.astype(np.float32))
    end_points = dict()
    cloud_sampled = torch.from_numpy(cloud_sampled[np.newaxis].astype(np.float32))
    device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
    cloud_sampled = cloud_sampled.to(device)
    end_points['point_clouds'] = cloud_sampled
    end_points['cloud_colors'] = color_sampled

    return end_points, cloud


def get_grasps(net, end_points):
    # 送入网络进行处理
    with torch.no_grad():
        end_points = net(end_points)
        grasp_preds, approachings, angles = pred_decode(end_points)
    gg_array = grasp_preds[0].detach().cpu().numpy()
    gg = GraspGroup(gg_array)
    return gg, approachings.cpu().numpy(), angles.cpu().numpy()


def collision_detection(gg, cloud, approachings, angles):
    mfcdetector = ModelFreeCollisionDetector(cloud, voxel_size=voxel_size)
    collision_mask = mfcdetector.detect(gg, approach_dist=0.05, collision_thresh=collision_thresh)
    gg = gg[~collision_mask]
    approachings = approachings[~collision_mask]
    angles = angles[~collision_mask]
    return gg, approachings, angles


def vis_grasps(gg: GraspGroup, cloud):
    gg.nms()
    gg.sort_by_score()
    gg = gg[:10]
    grippers = gg.to_open3d_geometry_list()
    o3d.visualization.draw_geometries([cloud, *grippers])


class GN1BMinimal:
    def __init__(self, cam_param, n_grasp=10, ckpt_filename='') -> None:
        self.cam_param = cam_param
        self.intrinsics = self.cam_param.to_matrix()
        self.depth_scale = self.cam_param.scale

        ########################### 载入网络 ###########################
        self.net = GraspNet(input_feature_dim=0, num_view=num_view, num_angle=12, num_depth=4,
                    cylinder_radius=0.05, hmin=-0.02, hmax_list=[0.01, 0.02, 0.03, 0.04], is_training=False)
        device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
        self.net.to(device)
        # Load checkpoint
        if ckpt_filename == '':
            ckpt_filename = ckpt_path
        checkpoint = torch.load(ckpt_filename)
        self.net.load_state_dict(checkpoint['model_state_dict'])
        start_epoch = checkpoint['epoch']
        track_log("-> loaded checkpoint %s (epoch: %d)" % (ckpt_filename, start_epoch))
        # set model to eval mode
        self.net.eval()
        self.n_grasp = n_grasp
        self.camera = CameraInfo(width=640.0, height=480.0, 
                                fx=self.intrinsics[0][0], fy=self.intrinsics[1][1], 
                                cx=self.intrinsics[0][2], cy=self.intrinsics[1][2], 
                                scale=self.depth_scale)

    def detect(self, rgb, depth):
        end_points, cloud = make_prediction(rgb, depth, self.camera)
        # 得到一组抓取姿态, 从中得到最终的抓取点和对应的抓取姿态
        grasp_group, approachings, angles = get_grasps(self.net, end_points)
        n_grasp = min(self.n_grasp, len(grasp_group))
        if n_grasp == 0:
            raise GraspNotFoundError('No grasp found in given scene!!!')

        # 全部按照得分排序
        _ = grasp_group.sort_by_score()
        score = grasp_group.grasp_group_array[:, 0]
        index = np.argsort(score)
        # 得分高的在排在前面
        index = index[::-1]
        grasp_group.grasp_group_array = grasp_group.grasp_group_array[index]
        approachings = approachings[index]
        angles = angles[index]

        if collision_thresh > 0:
            # 如果设置了碰撞检测，则进行碰撞检测移除有碰撞的抓取位姿
            grasp_group, approachings, angles = collision_detection(grasp_group, np.array(cloud.points), approachings, angles)

        # 取得分高的前self.n_grasp个抓取作为候选
        grasp_group = grasp_group[:self.n_grasp]
        approachings = approachings[:self.n_grasp]
        angles = angles[:self.n_grasp]
        # 先抓取最靠近相机的物体
        # 按照translation的z分量从小到大进行排序
        grasp_arr  = copy.deepcopy(grasp_group.grasp_group_array)
        translation_z_position = 15
        z = grasp_arr[:, translation_z_position]
        index = np.argsort(z)
        grasp_arr = grasp_arr[index]
        approachings = approachings[index]
        angles = angles[index]

        # 选最靠近相机的那个作为目标抓取
        grasp_candidate_arr = grasp_arr[0]
        grasp_candidate_approaching = approachings[0]
        grasp_candidate_angle = angles[0]
        grasp_candidate_score = grasp_candidate_arr[0]
        grasp_candidate_width = grasp_candidate_arr[1]
        grasp_candidate_depth = grasp_candidate_arr[3]
        grasp_candidate_rot = grasp_candidate_arr[4: 13].reshape(3, 3)
        grasp_candidate_trans = grasp_candidate_arr[13: 16]
        grasp_candidate_center = grasp_candidate_trans + grasp_candidate_rot[:, 0] * grasp_candidate_depth
        # 预抓取点的计算,抓取点的抓取方向反向走20cm
        pre_grasp_point = grasp_candidate_trans - grasp_candidate_rot[:, 0] * 0.20

        return grasp_candidate_rot, grasp_candidate_trans, grasp_candidate_center, grasp_candidate_width, \
                grasp_candidate_score, pre_grasp_point, grasp_candidate_approaching, grasp_candidate_angle
