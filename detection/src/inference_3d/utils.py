# coding=utf-8

import numpy as np
import open3d as o3d


def from_ndarray_to_o3d_pc(pts):
    """
    从数组表示的点云转换为open3d格式的点云
    :param pts:
    :return:
    """
    assert pts.shape[1] == 3, 'pts shape must be (n, 3)'
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


def estimate_normals(cloud_xyz, fast_normal_computation=True):
    """
    通过open3d计算输入点云的法向量

    :return: np.ndarray(num_points, 3)的法向量
    """
    cloud_o3d = from_ndarray_to_o3d_pc(cloud_xyz)
    search_option = o3d.geometry.KDTreeSearchParamHybrid(radius=0.005, max_nn=30)
    cloud_o3d.estimate_normals(search_param=search_option, fast_normal_computation=fast_normal_computation)
    
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


def downsample(cloud, keep_ratio=0.8):
    """
    点云下采样操作, 保留keep_ratio比例的点数
    """
    ...