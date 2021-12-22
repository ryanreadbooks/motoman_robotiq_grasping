"""
written by ryanreadbooks
date: 2021/11/27
"""
# coding=utf-8
from typing import Dict
import torch
from skimage.filters import gaussian
from skimage.feature import peak_local_max
import numpy as np
import copy

from .key_constants_pool import *
from .grasp_repr import GraspInLine


def pack_single_image(img: np.ndarray = None, depth: np.ndarray = None, out_size=384, device=torch.device('cuda')):
    """
    处理单张图片，处理成能够直接输入到网络的tensor，包括归一化预处理
    :param img: (h,w,3), np.ndarray
    :param depth: (h,w), np.ndarray，单位m
    :param out_size: 输出尺寸，默认截取输入图片中间的out_size部分区域
    :return: ret : dict with only KeyNetInput item
    """
    if img is None and depth is None:
        raise ValueError('img and depth can not be both None')
    if img is not None:
        # 需要对相应区域的裁剪
        h, w = img.shape[:2]
        left = w // 2 - out_size // 2
        top = h // 2 - out_size // 2
        img_crop = img[top: top + out_size, left: left + out_size]
        img = img_crop.astype(np.float32) / 255.0
        img -= img.mean()
        img = img.transpose((2, 0, 1))  # 转成[c,h,w]格式
        # 变成tensor
        img_tensor = torch.from_numpy(img)[None].to(device)  # (1,c,h,w)
        return {KeyNetInput: img_tensor, KeyOriginalImage: img_crop}
    else:
        h, w = depth.shape
        left = w // 2 - out_size // 2
        top = h // 2 - out_size // 2
        depth = depth[top: top + out_size, left:left + out_size]
        depth = np.clip(depth - depth.mean(), -1, 1)
        depth_tensor = torch.from_numpy(depth)[None].to(device)  # (1,h,w)
        return {KeyNetInput: depth_tensor, KeyDepthImg: depth}


def decode_net_output(pred: Dict, classification=True):
    """
    对网络的预测结果进行后处理
    :param pred: 包含了三个输出结果的字典 batchsize=1，内含在device上的torch.Tensor
    :param classification:
    :return: pred，新增进解码结果
    """
    graspness = pred[KeyGraspnessOut]  # (1,1,h,w)
    angle = pred[KeyGraspAngleOut]  # (1,1+n_cls,h,w)
    width = pred[KeyGraspWidthOut]  # (1,1,h,w)

    graspness = gaussian(graspness.squeeze().sigmoid().cpu().numpy(), sigma=2.0, preserve_range=True)  # (h,w)
    # graspness = graspness.squeeze().sigmoid().cpu().numpy()  # (h,w)
    width = width.squeeze().cpu().sigmoid().numpy()  # (h,w), 这里的width是归一化了的
    if classification:
        angle = torch.softmax(angle, dim=1)  # (1,1+n_cls,h,w)
        angle = torch.argmax(angle, dim=1).squeeze().cpu().numpy()  # (h,w)
    else:
        # 输出的是用regression得到的连续角度值
        # angle (1,1,h,w)
        angle = angle.squeeze()  # (h,w)

    width = gaussian(width, sigma=1.0, preserve_range=True)

    pred[KeyGraspnessPred] = graspness
    pred[KeyGraspAnglePred] = angle
    pred[KeyGraspWidthPred] = width

    return pred


def decode_graspmap(graspness, angle, width, original_img, angles_table, n_grasp=5, threshold=0.55, mode='global'):
    """
    解析grasp map，得到一系列的GraspInLine
    :param graspness: (h,w)
    :param angle: (h,w)
    :param width: (h,w)
    :param original_img: (h,w,3)
    :param n_grasp: 要从grasp map中选出多少个grasp的最大值
    :param threshold: 概率的阈值
    :param n_cls: 角度分类的数目
    :param mode: all 或者 peak，all从全局中选取n个大于threshold的grasp point；peak选取局部最大的grasp
    :return:
    """
    assert mode in ('global', 'local'), "mode should be either 'global' or 'local', {:s} not supported.".format(mode)
    h, w = graspness.shape

    if mode == 'global':
        # 所有超过阈值的点都要
        r, c = np.where(graspness >= threshold)
        pred_pts = np.vstack([r, c]).T
        # 只取top-n_grasp进行验证
        idx = np.argsort(graspness[r, c])
        pred_pts = pred_pts[idx[::-1]]
        pred_pts = pred_pts[:n_grasp]  # 只取前n个作为输出
    else:
        # 多个物体的时候，局部最大值的都取
        pred_pts = peak_local_max(image=graspness, min_distance=2, threshold_abs=threshold, num_peaks=n_grasp)
    original_img_for_drawing = copy.deepcopy(original_img)
    for (r, c) in pred_pts:
        angle_value = angles_table[angle[r, c]]  # rad
        width_value = width[r, c] * 200  # pixel
        grasp = GraspInLine(x=c, y=r, width=width_value, angle=angle_value, shape=(h, w), corners=None)
        original_img_for_drawing = grasp.draw_on_img(original_img_for_drawing)

    return original_img_for_drawing, pred_pts
