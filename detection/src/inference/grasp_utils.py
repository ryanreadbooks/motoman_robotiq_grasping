# coding=utf-8
"""
written by ryanreadbooks
date: 2021/11/25
"""
from typing import List, Tuple

import numpy as np
from .grasp_repr import GraspInLine
from skimage.draw import line_aa


def _gr_text_to_no(line, scale=1.0, offset=(0, 0)):
    """
    将cornell标注文件中的一行转化为一个点
    :param line: 文件中的一行
    :param offset: 施加的偏移量
    :return: Point [y, x]，返回的点，格式是先y后x
    """
    x, y = line.split()
    return [int(round(float(y) / scale)) - offset[0], int(round(float(x) / scale)) - offset[1]]


def parse_cornell_label_file(fname, im_shape, scale=1.0) -> List[GraspInLine]:
    """
    从cornell数据集中的标注文件中读取对应的抓取矩形框信息
    :param fname: 文件路径
    :param im_shape: 图片尺寸
    :param scale: 点的缩放系数
    :return: 包含所有抓取矩形框
    """
    grs: List[GraspInLine] = []
    with open(fname) as f:
        while True:
            # 一次性读入行，分别代表4个点，从左上角开始按照顺时针顺序排列
            p0 = f.readline()
            if not p0:
                break  # EOF
            p1, p2, p3 = f.readline(), f.readline(), f.readline()
            try:
                # 矩形框的4个点
                # shape=(4,2)
                gr = np.array([
                    _gr_text_to_no(p0, scale),
                    _gr_text_to_no(p1, scale),
                    _gr_text_to_no(p2, scale),
                    _gr_text_to_no(p3, scale)
                ])

                grs.append(grasp_rect_corners_2_grasp_line_repr(gr, im_shape))

            except ValueError:
                # 有些可能存在异常值
                continue
    return grs


def grasp_rect_corners_2_grasp_line_repr(corners: np.ndarray, im_shape: Tuple) -> GraspInLine:
    """
    将用四个顶点坐标表示的抓取矩形框转换成用直线表示的抓取
    :param corners: 四个顶点，np.ndarray[4,2]，先y后x
    :param im_shape: 图片尺寸
    :return:
    """
    # p1 *-----*-----* p2
    #        center
    p1 = (corners[0] + corners[3]) / 2  # 左边的点
    p2 = (corners[1] + corners[2]) / 2  # 右边的点
    center = (p1 + p2) / 2  # (y,x)
    width = np.linalg.norm(corners[1] - corners[0])  # 抓取宽度，单位为像素
    # 抓取角度，由于对称性，需要保证在[0,+pi]之间
    # 并且0和pi是对称的, angle at 0 = angle at pi
    dx = p2[1] - p1[1]
    dy = p2[0] - p1[0]
    # 定义为与图像x轴正方向的夹角
    # arctan(斜率) = angle
    angle = (np.arctan2(dy, dx) + np.pi / 2) % np.pi + np.pi / 2
    # 确保angle是在[0,pi]内
    if angle > np.pi:
        angle -= np.pi
    return GraspInLine(center[1], center[0], width, angle, im_shape, corners)


def draw_rect_grasp_on_img(img, rect_corners: np.ndarray):
    """
    在指定图片上绘制抓取矩形框
               line1
           c1---------c2
      line4 |         |  line2
            |         |
           c4---------c3
               line3
    :param img: 指定图片, np.ndarray, shape = (h,w) or (h,w,3)
    :param rect_corners: 抓取矩形框的4个顶点，shape=(4,2), [y,x]. 四个点的顺序为顺时针,
    :return:
    """
    assert rect_corners.shape == (4, 2), f'rect_corners should be of shape (4,2), but got {rect_corners.shape}'
    c1, c2, c3, c4 = rect_corners
    c1 = c1.astype(np.int)
    c2 = c2.astype(np.int)
    c3 = c3.astype(np.int)
    c4 = c4.astype(np.int)
    line1_r, line1_c, _ = line_aa(*c1, *c2)
    line2_r, line2_c, _ = line_aa(*c2, *c3)
    line3_r, line3_c, _ = line_aa(*c3, *c4)
    line4_r, line4_c, _ = line_aa(*c4, *c1)

    img[line1_r, line1_c] = np.array([0, 255, 0])
    img[line2_r, line2_c] = np.array([255, 0, 0])
    img[line3_r, line3_c] = np.array([0, 255, 0])
    img[line4_r, line4_c] = np.array([255, 0, 0])

    return img
