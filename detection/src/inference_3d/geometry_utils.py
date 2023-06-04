"""
一些关键函数
"""

import math
import cv2
import numpy as np
import pytransform3d.rotations as pyrot

from .depth_map_utils import fill_in_fast


def compute_rotmat_between_two_directions(v_from, v_to):
    """
    计算从v_from转换到v_to的旋转变换矩阵
    :param v_from: 起始向量
    :param v_to: 目标向量
    :return: 3x3旋转矩阵
    """

    return pyrot.matrix_from_rotor(pyrot.rotor_from_two_directions(v_from, v_to))


def align_camera_optical_to(direction: np.ndarray, point: np.ndarray, keep_dist=True):
    """
    将相机的光轴对准特定的方向,并且相机原点移动到point位置
    :param direction: 需要对齐的向量(相机坐标系下表示), np.ndarray(3,)
    :param point: 方向向量的起点(相机坐标系下表示)
    :param keep_dist: 对齐后是否保持原来相机和point的距离不变
    :return: 将相机变换过去需要进行的4x4的变换矩阵
    """

    direction = direction.squeeze()
    assert direction.shape == (3,), 'direction shape invalid'
    dist = np.linalg.norm(point)
    if keep_dist:
        translation = point + dist * -direction
    else:
        translation = point
    rot_diff = compute_rotmat_between_two_directions(np.array([0., 0., 1.]), direction)
    T = np.zeros((4, 4), dtype=np.float32)
    T[:3, :3] = rot_diff
    T[:3, -1] = translation
    T[3, 3] = 1.
    return T


def transform_point_cloud(cloud, transform, fmt='4x4'):
    """
    用变换矩阵变换点云
    :param cloud: 点云 shape=(n,3)
    :param transform: (3,3)/(3,4)/(4,4)的ndarray
    :param fmt: string '3x3'/'3x4'/'4x4'
                '3x3' --> 旋转矩阵
                '3x4'/'4x4' --> 旋转矩阵 + 平移量
    :return: np.ndarray shape=(n,3) 变换之后的点云
    """

    if not (fmt == '3x3' or fmt == '4x4' or fmt == '3x4'):
        raise ValueError('Unknown transformation format, only support \'3x3\' or \'4x4\' or \'3x4\'.')
    if fmt == '3x3':
        cloud_transformed = np.dot(transform, cloud.T).T
    elif fmt == '4x4' or fmt == '3x4':
        ones = np.ones(cloud.shape[0])[:, np.newaxis]
        cloud_ = np.concatenate([cloud, ones], axis=1)
        cloud_transformed = np.dot(transform, cloud_.T).T
        cloud_transformed = cloud_transformed[:, :3]
    else:
        raise ValueError(f'not supported fmt {fmt}')
    return cloud_transformed


def uniform_vectors_within_cone(phi=np.pi / 4, n=15):
    """
    在圆锥内均匀采样生成n个单位向量
    math: v = \left[ \sqrt{1-z^2}\cos{\theta}, \sqrt{1-z^2}\sin{\theta},z \right] \\
            where\ \theta \in \left[ 0, 2\pi \right)\ z \in \left[\cos\phi, 1\right) \\
            \phi\in\left(0, \frac{\pi}{2}\right)
    :param phi: 圆锥的轴与母线之间的夹角
    :param n: 生成的单位向量数量
    :return: np.ndarray shape=(n, 3) 随机得到的n个单位向量 
    """

    z = np.random.uniform(np.cos(phi), 1, n)
    theta = np.random.uniform(0, 2 * np.pi, n)
    vx = np.sqrt(1 - z ** 2) * np.cos(theta)
    vy = np.sqrt(1 - z ** 2) * np.sin(theta)
    vz = z
    return np.vstack([vx, vy, vz]).T


def project_pc_back_to_plane(cloud: np.ndarray, intrinsics: np.ndarray, im_size=(640, 480), has_color=False, fill_empty=True):
    """
    使用相机内参将点云重新投影回平面,得到rgb或者depth图像
    :param cloud: 待重投影的点云, shape = (n, 3) 如果has_color==True shape = (n, 6) [xyzrgb]
    :param im_size: 平面的尺寸 (w, h)
    :param intrinsics: 相机内参, shape = (3,3)
    :param has_color: 点云中是否包含颜色
    :param fill_empty: 生成的图片是否需要填充确实的像素
    :return: 生成的深度图,shape=im_size; 如果有颜色,同时返回深度图和彩色图(h,w,3)
    """
    
    if not has_color:
        assert len(cloud.shape) == 2 and cloud.shape[1] == 3 and intrinsics.shape == (3, 3), 'invalid input args with wrong shapes'
    else:
        assert len(cloud.shape) == 2 and cloud.shape[1] == 6 and intrinsics.shape == (3, 3), 'invalid input args with wrong shapes'
    w, h = im_size
    cloud_xyz = cloud[:, :3]
    fabricated_depth = np.zeros((h, w), dtype=np.float32)
    fx, fy = intrinsics[0][0], intrinsics[1][1]
    cx, cy = intrinsics[0][2], intrinsics[1][2]
    v = (fy * cloud_xyz[:, 1] / cloud_xyz[:, 2] + cy)
    u = (fx * cloud_xyz[:, 0] / cloud_xyz[:, 2] + cx)
    valid_idx = np.logical_and(np.logical_and(v >= 0, v < h), np.logical_and(u >= 0, u < w))  # make sure indices do not overflow
    u = np.floor(u[valid_idx]).astype(np.uint)
    v = np.floor(v[valid_idx]).astype(np.uint)
    fabricated_depth[v, u] = cloud_xyz[:, 2][valid_idx]
    fabricated_color = None
    if has_color:
        cloud_color = cloud[:, 3:]
        fabricated_color = np.zeros((h, w, 3), dtype=np.uint8)
        fabricated_color[v, u, :] = cloud_color[valid_idx]
    if fill_empty:
        # fabricated_depth = cv2.morphologyEx(fabricated_depth, cv2.MORPH_DILATE,
        #                                     cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=1)
        fabricated_depth = fill_in_fast(fabricated_depth, extrapolate=True)
        if has_color:
            # 三个通道分别膨胀操作
            r = cv2.morphologyEx(fabricated_color[:, :, 0], cv2.MORPH_DILATE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)    # (h,w)
            g = cv2.morphologyEx(fabricated_color[:, :, 1], cv2.MORPH_DILATE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)    # (h,w)
            b = cv2.morphologyEx(fabricated_color[:, :, 2], cv2.MORPH_DILATE,
                                 cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2)    # (h,w)
            fabricated_color = np.concatenate([r[:, :, None], g[:, :, None], b[:, :, None]], axis=2)    # (h,w,3)
    if has_color:
        return fabricated_depth, fabricated_color
    return fabricated_depth
