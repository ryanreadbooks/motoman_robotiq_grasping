# coding=utf-8
"""
written by ryanreadbooks
date: 2021/11/26
"""

import numpy as np
from skimage.draw import polygon
import os
from PIL import Image, ImageOps
import open3d as o3d


# fixme time consuming
def arg_thresh(array, thresh):
    """
    获取array中大于thresh的二维索引
    :param array: 二维array
    :param thresh: float阈值
    :return: array shape=(n, 2)
    """
    res = np.where(array > thresh)
    rows = np.reshape(res[0], (-1, 1))
    cols = np.reshape(res[1], (-1, 1))
    locs = np.hstack((rows, cols))
    for i in range(locs.shape[0]):
        for j in range(locs.shape[0])[i + 1:]:
            if array[locs[i, 0], locs[i, 1]] < array[locs[j, 0], locs[j, 1]]:
                locs[[i, j], :] = locs[[j, i], :]

    return locs


def polygon_iou(polygon_a, polygon_b):
    """
    计算两个多边形的IOU
    :param polygon_a: [[row1, col1], [row2, col2], ...]
    :param polygon_b: 同上
    :return:
    """
    rr1, cc1 = polygon(polygon_b[:, 0], polygon_b[:, 1])
    rr2, cc2 = polygon(polygon_a[:, 0], polygon_a[:, 1])

    try:
        r_max = max(rr1.max(), rr2.max()) + 1
        c_max = max(cc1.max(), cc2.max()) + 1
    except:
        return 0

    canvas = np.zeros((r_max, c_max))
    canvas[rr1, cc1] += 1
    canvas[rr2, cc2] += 1
    union = np.sum(canvas > 0)
    if union == 0:
        return 0
    intersection = np.sum(canvas == 2)
    return intersection / union


def ensure_dirs(dir_path):
    if not os.path.exists(dir_path):
        os.makedirs(dir_path)


def add_board_and_save(ret, fname):
    im = ret.to_rgba(ret._A[::-1] if ret.origin == 'lower' else ret._A, bytes=True, norm=True)
    im_pil = Image.fromarray(im)
    ImageOps.expand(im_pil, border=5, fill='black').save(fname)


def raw_generate_pc(rgb, depth, cam_intrin, depth_scale=1.0, depth_trunc=1.5):
    """
    给定rgb和depth生成点云
    :param rgb: o3d.geometry.Image或者np.ndarray
    :param depth: o3d.geometry.Image或者np.ndarray，单位不需要转换，配合camera_config进行单位转换
    :param camera_config: 相机配置参数
    :return:
    """
    h, w, _ = np.asarray(rgb).shape
    if isinstance(rgb, np.ndarray) and isinstance(depth, np.ndarray):
        assert rgb.shape[:2] == depth.shape, 'Size of rgb and depth do not match.'
        h, w, _ = rgb.shape
        rgb = o3d.geometry.Image(rgb.astype(np.uint8))
        depth = o3d.geometry.Image(depth.astype(np.uint16))

    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color=rgb, depth=depth,
        depth_scale=depth_scale, depth_trunc=depth_trunc,
        convert_rgb_to_intensity=False
    )
    cx = cam_intrin[0, 2]
    cy = cam_intrin[1, 2]
    fx = cam_intrin[0, 0]
    fy = cam_intrin[1, 1]
    camera_in = o3d.camera.PinholeCameraIntrinsic(width=w, height=h, fx=fx, fy=fy, cx=cx, cy=cy)
    cloud = o3d.geometry.PointCloud.create_from_rgbd_image(
        image=rgbd, intrinsic=camera_in
    )

    return cloud


def angle_to_rot_mat(rx, ry, rz, rot_type='abg'):
    """
    绕三个轴的旋转角度化成一个旋转矩阵
    :param rx: 绕x轴的旋转量，单位(rad)
    :param ry: 绕y轴的旋转量，单位(rad)
    :param rz: 绕z轴的旋转量，单位(rad)
    :param rot_type: 设置三个旋转角度化成旋转矩阵的形式，可选'abg'和'gba'。其中abg的三次旋转都是绕定轴的；gba的三次旋转是绕动轴的。
    :return:
    """

    rotation_x = np.array([[1, 0, 0],
                           [0, np.cos(rx), -np.sin(rx)],
                           [0, np.sin(rx), np.cos(rx)]])

    rotation_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                           [0, 1, 0],
                           [-np.sin(ry), 0, np.cos(ry)]])

    rotation_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                           [np.sin(rz), np.cos(rz), 0],
                           [0, 0, 1]])

    if rot_type == 'abg':
        rot = (rotation_z @ rotation_y) @ rotation_x  # 'abg'
    else:
        rot = (rotation_x @ rotation_y) @ rotation_z  # 'gba'

    return rot
