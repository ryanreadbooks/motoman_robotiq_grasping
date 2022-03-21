from base64 import decode
from pyexpat import model
from importlib_metadata import version
import numpy as np
import torch

from .models import PointNet, decode_pointnet_output
from .utils import *


def detection3d_log(msg):
    print(f"node_detection_3d::pipeline => {msg}")


class PointNetGraspnessDetector:
    def __init__(self, model_path: str, version=1):
        self.model_path = model_path
        self.version = version
        # 加载模型
        self.device = torch.device("cuda")
        if version == 1:
            self.model = PointNet(num_class=2)
        elif version == 2:
            # TODO pointnet++ model
            pass
        # ! for development and use model path in checkpoint temporarily, modify it later
        checkpoint = torch.load(model_path)
        self.model.load_state_dict(checkpoint['key_state_dict_param'])
        self.model.to(self.device)
        self.model.eval()

    def detect(self, cloud_xyz):
        """对输入的点云进行graspness和objectness的检测

        Args:
            cloud_xyz (torch.tensor): 输入点云, shape=(1, num_points, 3)

        Returns:
            np.ndarray: cloud_xyz
            np.ndarray: cloud_normals
        """
        assert len(cloud_xyz.shape) == 3 and cloud_xyz.shape[2] == 3, "make sure cloud_xyz shape is (1, num_points, 3), but got {}".format(
            cloud_xyz.shape)
        detection3d_log(f'cloud_xyz shape = {cloud_xyz.shape}')
        pred = self.model(cloud_xyz.transpose(2, 1))
        graspness, objectness = decode_pointnet_output(pred)  # (num_points,)

        return graspness, objectness


class SixDoFGraspDetector:
    def __init__(self, model_path, graspness_detector_version=1):
        self.num_points = 20000
        self.version = graspness_detector_version
        self.graspness_detector = PointNetGraspnessDetector(
            model_path=model_path, version=self.version
        )

    def detect(self, cloud):
        """从输入场景点云中得到物体的抓取位姿

        Args:
            cloud (np.ndarray): 输入点云, shape=(num_points, 3)
        """

        # sample to num_points = 20000 to match the network input
        if len(cloud) >= self.num_points:
            idxs = np.random.choice(len(cloud), self.num_points, replace=False)
        else:
            idxs1 = np.arange(len(cloud))
            idxs2 = np.random.choice(len(cloud), self.num_points - len(cloud), replace=True)
            idxs = np.concatenate([idxs1, idxs2], axis=0)

        cloud_xyz = cloud[idxs]

        cloud_normals = estimate_normals(cloud_xyz)  # (num_points, 3)
        cloud_input = torch.from_numpy(pc_normalize(cloud_xyz))[None].to(self.graspness_detector.device) # make shape=(1, num_points, 3)
        detection3d_log('cloud normals shape={}, cloud_xyz shape={}, cloud_input_shape='.format(cloud_normals.shape, cloud_xyz.shape, cloud_input.shape))

        graspness, objectness = self.graspness_detector.detect(cloud_input.to(torch.float32))
        graspable_mask = np.logical_and(graspness, objectness)
        # select cloud by graspness and objectness
        cloud_xyz = cloud_xyz[graspable_mask]
        cloud_normals = cloud_normals[graspable_mask]

        # ! for experiment
        return cloud_xyz