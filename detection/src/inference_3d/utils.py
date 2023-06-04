# coding=utf-8

import numpy as np
import open3d as o3d
import torch

_search_option = o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30)

def from_ndarray_to_o3d_pc(pts):
    """
    从数组表示的点云转换为open3d格式的点云
    :param pts:
    :return:
    """
    assert pts.shape[1] == 3, f'pts shape must be (n, 3), but got {pts.shape}'
    return o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts))


def select_pc_by_index(pc, index, invert=False):
    """
    从pc中提取出index的点云
    """
    if o3d.__version__ == '0.9.0.0':
        return pc.select_down_sample(index, invert)
    return pc.select_by_index(index, invert)


def pc_normalize(pc):
    """
    对点云进行归一化处理
    """
    l = pc.shape[0]
    centroid = np.mean(pc, axis=0)
    pc = pc - centroid
    m = np.max(np.sqrt(np.sum(pc ** 2, axis=1)))
    pc = pc / m
    return pc


def normalize_cloud_color(color):
    """对颜色进行归一化处理

    Args:
        color (np.ndarray (n,3)): 颜色矩阵
    """
    color_std, color_mean = np.std(color), np.mean(color)
    color_normalized = (color - color_mean) / color_std
    return color_normalized


def estimate_normals(cloud_xyz, fast_normal_computation=True):
    """
    通过open3d计算输入点云的法向量

    :return: np.ndarray(num_points, 3)的法向量
    """
    if type(cloud_xyz) == np.ndarray:
        cloud_o3d = from_ndarray_to_o3d_pc(cloud_xyz)
    else:   # o3d.geometry.PointCloud
        cloud_o3d = cloud_xyz
    cloud_o3d.estimate_normals(search_param=_search_option, fast_normal_computation=fast_normal_computation)
    
    cloud_o3d.normalize_normals()
    cloud_o3d.orient_normals_towards_camera_location()  # 法向量指向相机位置

    return np.asarray(cloud_o3d.normals)


def segment_plane(scene_cloud: np.ndarray):
    """
    平面分割
    scene_cloud: 输入点云, shape=(n,3)
    返回去除了平面之后的点云
    """
    scene_cloud = from_ndarray_to_o3d_pc(scene_cloud)
    _, plane_idx = scene_cloud.segment_plane(
        distance_threshold=0.01, ransac_n=5, num_iterations=50
    )
    return np.asarray(
        select_pc_by_index(scene_cloud, plane_idx, invert=True).points
    )


def random_sample(cloud, num_points):
    """点云下采样操作, 随机提取出num_points个点

    Args:   
        cloud (np.ndarray shape=(n,3)): 输入点云
        num_points (int): 需要随机采样的点数 

    Returns:
        np.ndarray: 随机采样得到的点的索引
    """
    if len(cloud) >= num_points:
        idxs = np.random.choice(len(cloud), num_points, replace=False)
    else:
        idxs1 = np.arange(len(cloud))
        idxs2 = np.random.choice(len(cloud), num_points - len(cloud), replace=True)
        idxs = np.concatenate([idxs1, idxs2], axis=0)
    
    return idxs


def create_point_cloud_from_depth_image(depth, camera, organized=True):
    """
    从深度图生成点云
    
    Args:
        depth: (np.ndarray, shape=(h,w)): 深度图
        camera: (CameraParams): 相机内参矩阵
        organized: (bool): 是否保持输出的点云(h,w,3)的形状, 如果为false,则返回(h*w, 3)形状的矩阵
    """
    assert (depth.shape[0] == camera.height and depth.shape[1] == camera.width)
    xmap = np.arange(camera.width)
    ymap = np.arange(camera.height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    points_z = depth / camera.scale
    points_x = (xmap - camera.cx) * points_z / camera.fx
    points_y = (ymap - camera.cy) * points_z / camera.fy
    cloud = np.stack([points_x, points_y, points_z], axis=-1)
    if not organized:
        cloud = cloud.reshape([-1, 3])
    return cloud


def rotmat_from_direction_angle(direction, angle):
    """给定一个方向向量和一个绕着方向向量的旋转角度，计算出旋转矩阵

    Args:
        direction (NdArray): 方向向量
        angle (float, rad): 旋转角度

    Returns:
        rotation_matrix: 转换后的旋转矩阵
    """
    axis_x = direction
    ones = np.ones(axis_x.shape[0], dtype=np.float32)
    zeros = np.zeros(axis_x.shape[0], dtype=np.float32)
    axis_y = np.stack([-axis_x[:, 1], axis_x[:, 0], zeros], axis=-1)    # 构造和axis_x垂直的向量
    mask_y = (np.linalg.norm(axis_y, axis=-1) == 0)
    axis_y[mask_y, 1] = 1
    axis_x = axis_x / np.linalg.norm(axis_x, axis=-1, keepdims=True)
    axis_y = axis_y / np.linalg.norm(axis_y, axis=-1, keepdims=True)
    axis_z = np.cross(axis_x, axis_y)
    sin = np.sin(np.array([angle]))
    cos = np.cos(np.array([angle]))
    R1 = np.stack([ones, zeros, zeros, zeros, cos, -sin, zeros, sin, cos], axis=-1) # 绕x轴旋转angle弧度的旋转矩阵
    R1 = R1.reshape([-1, 3, 3])
    # 构建出从初始位置到与axis_x对齐所需的旋转矩阵
    R2 = np.stack([axis_x, axis_y, axis_z], axis=-1)
    rotation_matrix = np.matmul(R2, R1)[0]  # 先进行R2的旋转变换，随后在变换后的基础上进行R1的旋转变换，所以右乘

    return rotation_matrix