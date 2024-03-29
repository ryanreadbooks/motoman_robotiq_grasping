# coding=utf-8
"""
written by ryanreadbooks
date: 2021/11/25
function: 抓取的表示类实现，有基于矩形框的表示和基于直线的表示
"""
from typing import Tuple

import numpy as np
import open3d as o3d
from skimage.draw import disk, line_aa
from pytransform3d import rotations as pr, transformations as pt


class GraspCenter:
    """
    抓取点中心
    """

    def __init__(self, x, y):
        self.x = x
        self.y = y


class GraspInLine:
    """
    用直线表示的抓取
    """

    def __init__(self, x, y, width, angle, shape, corners):
        """
        抓取表示
        :param x: in pixel
        :param y: in pixel
        :param width: in pixel
        :param angle: in radian
        """
        self.center: GraspCenter = GraspCenter(x, y)
        self.width = width
        assert angle >= 0, f'grasp angle should be >= 0, but got {angle}'
        self.angle = angle
        self.shape = shape
        self.rr, self.cc = disk(center=(self.center.y, self.center.x), radius=self.width / 3, shape=shape)
        self.corners = corners

    @property
    def normalized_width(self):
        width = min(self.width, 200)  # 抓取最大为200像素
        return width / 200  # 归一化到[0,1]范围内

    def provide_graspness_area(self) -> np.ndarray:
        """
        返回这个抓取的可抓取区域：为以center为圆心，以width/3为半径的一个圆
        :return: graspness area, shape = shape
        """
        graspness_area = np.zeros(self.shape, dtype=np.uint8)
        graspness_area[self.rr, self.cc] = 1
        return graspness_area

    def provide_grasp_angle_cls(self, n_angle_cls: int = 120) -> np.ndarray:
        """
        返回这个抓取的角度表示形式, 离散形式
        :param n_angle_cls: 角度分类个数, 将[0,2pi]范围分成n_angle_cls份
        :return:
        """
        grasp_angle_cls = np.zeros((n_angle_cls, *self.shape), dtype=np.uint8)
        # 角度转成分类类别，类别编号从1开始
        len_interval = np.pi / n_angle_cls
        k = np.ceil(self.angle / len_interval).astype(np.uint8)
        grasp_angle_cls[k, self.rr, self.cc] = 1
        return grasp_angle_cls

    def provide_grasp_angle_reg(self) -> np.ndarray:
        """
        返回这个抓取的angle map, 连续数字
        :return:
        """
        grasp_angle_reg = np.zeros(self.shape, dtype=np.float32)
        grasp_angle_reg[self.rr, self.cc] = self.angle
        return grasp_angle_reg

    def provide_grasp_width_area(self) -> np.ndarray:
        """
        返回这个抓取的抓取宽度区域，位置和graspness一致
        :return: grasp width area, shape = shape
        """
        width_area = np.zeros(self.shape, dtype=np.float32)
        width = min(self.width, 200)  # 抓取最大为200像素
        width_area[self.rr, self.cc] = width / 200  # 归一化到[0,1]范围内
        return width_area

    def draw_on_img(self, img):
        r"""
        将grasp可视化到img上;
        用下述公式从width和center中恢复center旁边的两个点:
            \tan\left(angle\right)=\frac{y_0 - y_1}{x_0 - x_1} \\
            \left(x_0 - x_1\right)^2 + \left(y_0 - y_1\right)^2 = \left(\frac{w}{2}\right)^2
        :param img: 图片, np.ndarray, shape=(h,w,3)
        """
        assert self.shape == img.shape[0:2], f'make sure identical shapes, self.shape={self.shape}, img.shape={img.shape}'
        r = 3
        center_draw_r, center_draw_c = disk((self.center.y, self.center.x), r, shape=self.shape)
        # 根据angle得到另外两点
        # 垂直情况单独处理
        x1, x2, y1, y2 = self._cal_p1_p2()
        # print(f'DEBUG: x0 = {self.center.x}, y0 = {self.center.y} x1 = {x1}, y1 = {y1}, x2 = {x2}, y2 = {y2}')
        # print(f'DEBUG: grasp width = {self.width}, angle = {np.rad2deg(self.angle)} degrees ({self.angle} rad)')
        x1_draw_r, x1_draw_c = disk((int(y1), int(x1)), r, shape=self.shape)
        x2_draw_r, x2_draw_c = disk((int(y2), int(x2)), r, shape=self.shape)
        if img.ndim == 3:
            img[center_draw_r, center_draw_c] = np.array([255, 0, 0])
            img[x1_draw_r, x1_draw_c] = np.array([0, 255, 0])
            img[x2_draw_r, x2_draw_c] = np.array([0, 0, 255])
        else:
            img[center_draw_r, center_draw_c] = 1.0
            img[x1_draw_r, x1_draw_c] = 0.5
            img[x2_draw_r, x2_draw_c] = 0.0

        line_r, line_c, _ = line_aa(int(y1), int(x1), int(y2), int(x2))
        # 限制数值不要越界
        line_r = [min(i, self.shape[0] - 1) for i in line_r]
        line_c = [min(i, self.shape[1] - 1) for i in line_c]
        if img.ndim == 3:
            img[line_r, line_c] = np.array([238, 180, 34])
        else:
            img[line_r, line_c] = (238 + 180 + 34) / 255.0 / 3.0

        return img

    def _cal_p1_p2(self):
        """
        计算直线的两个端点的坐标
        :return:
        """
        if np.abs(np.rad2deg(self.angle) - 90.0) <= 1e-8:
            x1 = x2 = self.center.x
            y1 = self.center.y - self.width / 2
            y2 = self.center.y + self.width / 2
        else:
            k = np.tan(self.angle)
            tmp = (self.width / 2) * np.sqrt(1 / (1 + k ** 2))
            x1 = self.center.x - tmp
            x2 = self.center.x + tmp
            y1 = self.center.y - k * (self.center.x - x1)
            y2 = self.center.y - k * (self.center.x - x2)
        return x1, x2, y1, y2

    def rotate(self, degree, center):
        """
        以center为中心,将grasp旋转degree角度
        :param degree:
        :param center:
        :return:
        """
        with UpdateGraspRegion(self):
            angle = np.deg2rad(degree)
            R = np.array(
                [
                    [np.cos(-angle), np.sin(-angle)],
                    [-1 * np.sin(-angle), np.cos(-angle)],
                ]
            )
            c = np.array(center).reshape((1, 2))
            x1, x2, y1, y2 = self._cal_p1_p2()
            points = np.array([[y1, x1], [self.center.y, self.center.x], [y2, x2]])
            points = ((np.dot(R, (points - c).T)).T + c).astype(np.int)
            # 根据旋转结果反推出角度并且更新对应参数
            p1, p2 = points[0], points[2]
            self.angle = self.cal_angle_from_p1_p2(p1, p2)
            self.center.y, self.center.x = points[1][0], points[1][1]

    @staticmethod
    def cal_angle_from_p1_p2(p1, p2):
        dx = p2[1] - p1[1]
        dy = p2[0] - p1[0]
        # 定义为与图像x轴正方向的夹角
        # arctan(斜率) = angle
        angle = (np.arctan2(dy, dx) + np.pi / 2) % np.pi + np.pi / 2
        # 确保angle是在[0,pi]内
        if angle > np.pi:
            angle -= np.pi
        return angle

    def zoom(self, factor, center):
        """
        将grasp对应的进行缩放
        :param factor:
        :param center:
        :return:
        """
        with UpdateGraspRegion(self):
            sr = int(self.shape[0] * (1 - factor)) // 2  # 截取的开始
            sc = int(self.shape[1] * (1 - factor)) // 2  # 截取的结束
            # 先将center偏移
            self.center.y -= sr
            self.center.x -= sc

            T = np.array(
                [
                    [1 / factor, 0],
                    [0, 1 / factor]
                ]
            )
            c = np.array(center).reshape((1, 2))
            x1, x2, y1, y2 = self._cal_p1_p2()
            points = np.array([[y1, x1], [self.center.y, self.center.x], [y2, x2]])
            points = ((np.dot(T, (points - c).T)).T + c).astype(np.int)
            self.center.y, self.center.x = points[1, 0], points[1, 1]
            self.width = np.linalg.norm(points[0] - points[2])

    @staticmethod
    def to_rect_repr(x, y, angle, width):
        """
        将抓取中心点，抓取角度和抓取宽度三个元素转换成抓取矩形框表示形式
        :param x: 抓取点中心x坐标
        :param y: 抓取点中心y坐标
        :param angle: 抓取角度
        :param width: 抓取宽度
        :return: 抓取矩形框的4个顶点坐标 [[y1,x1],[y2,x2],[y3,x3],[y4,x4]]
        """
        # rect_height = width / 2
        # rect_height = width / 1.3
        rect_height = 30
        x1, x2, y1, y2 = GraspInLine(x, y, width, angle, None, None)._cal_p1_p2()
        off_x = rect_height / 2 * np.sin(angle)
        off_y = rect_height / 2 * np.cos(angle)

        # 四个角按照顺时针顺序排列
        cx1, cy1 = x1 + off_x, y1 - off_y
        cx2, cy2 = x2 + off_x, y2 - off_y
        cx3, cy3 = x2 - off_x, y2 + off_y
        cx4, cy4 = x1 - off_x, y1 + off_y

        return np.array([
            [cy1, cx1],
            [cy2, cx2],
            [cy3, cx3],
            [cy4, cx4],
        ])

    def offset_corners(self, left, top):
        self.corners[:, 0] -= top
        self.corners[:, 1] -= left

    def draw_rect_on_img(self, img):
        """
        在给定的图片上画出corners表示的rect的grasp
        :param img:
        :return:
        """
        y0, x0 = self.corners[0]
        y1, x1 = self.corners[1]
        y2, x2 = self.corners[2]
        y3, x3 = self.corners[3]
        line_r0, line_c0, _ = line_aa(int(y0), int(x0), int(y1), int(x1))
        line_r1, line_c1, _ = line_aa(int(y1), int(x1), int(y2), int(x2))
        line_r2, line_c2, _ = line_aa(int(y2), int(x2), int(y3), int(x3))
        line_r3, line_c3, _ = line_aa(int(y3), int(x3), int(y0), int(x0))

        img[line_r0, line_c0] = np.array([0, 255, 0])
        img[line_r2, line_c2] = np.array([0, 255, 0])
        img[line_r1, line_c1] = np.array([0, 0, 255])
        img[line_r3, line_c3] = np.array([0, 0, 255])

        return img


class UpdateGraspRegion:
    """
    统一更新GraspInLine.rr, GraspInLine.cc的上下文管理器
    """

    def __init__(self, grasp: GraspInLine):
        self.grasp = grasp

    def __enter__(self):
        pass

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.grasp.rr, self.grasp.cc = \
            disk(center=(self.grasp.center.y, self.grasp.center.x), radius=self.grasp.width / 3, shape=self.grasp.shape)


def check_equal(x, y, tol=1e-7):
    if isinstance(x, np.ndarray) and isinstance(y, np.ndarray):
        diff = x - y
        return np.mean(np.abs(diff)) < tol
    else:
        return np.abs(x - y) < tol


class GripperFrame:
    """
    表示坐标轴，包含三个坐标轴的方向向量
    """

    BaseX = np.array([1., 0., 0.])
    BaseY = np.array([0., 1., 0.])
    BaseZ = np.array([0., 0., 1.])
    ORIGIN = np.array([0., 0., 0.])

    def __init__(self, x, y, z, center: np.ndarray):
        """

        :param x: x轴向量
        :param y: y轴向量
        :param z: z轴向量
        :param center: 原点所在坐标
        """
        self._x = x
        self._y = y
        self._z = z
        self._check_orthogonal()

        self.center = center.reshape(3, )

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @property
    def z(self):
        return self._z

    @x.setter
    def x(self, new_x):
        align_R = GripperFrame._get_R_aligning_vectors(self._x, new_x)
        self._x = new_x
        self._y = align_R @ self._y
        self._z = align_R @ self._z

    @y.setter
    def y(self, new_y):
        align_R = GripperFrame._get_R_aligning_vectors(self._y, new_y)
        self._y = new_y
        self._x = align_R @ self._x
        self._z = align_R @ self._z

    @z.setter
    def z(self, new_z):
        align_R = GripperFrame._get_R_aligning_vectors(self._z, new_z)
        self._z = new_z
        self._x = align_R @ self._x
        self._y = align_R @ self._y

    @property
    def pose(self):
        return self.to_6dpose()

    @staticmethod
    def base_frame():
        return GripperFrame(GripperFrame.BaseX, GripperFrame.BaseY, GripperFrame.BaseZ, GripperFrame.ORIGIN)

    def _check_orthogonal(self, tol=3e-5):
        """
        检查是否三个轴是否垂直
        :return:
        """
        xy = self.x @ self.y
        xz = self.x @ self.z
        yz = self.y @ self.z

        if check_equal(xy, 0., tol) and check_equal(xz, 0., tol) and check_equal(yz, 0., tol):
            return
        else:
            raise ValueError(f'The given axes not orthogonal with each other, x={self.x}, y={self.y}, z={self.z}, '
                             f'xy={xy}, xz={xz}, yz={yz}')

    @staticmethod
    def init(center, axis, rot, rot_axis_name='z'):
        """
        根据给定的轴和旋转进行初始化
        :param axis:
        :param rot: 单位rad
        :param center:
        :return:
        """
        if rot_axis_name == 'z':
            firstR_ax_an = pr.axis_angle_from_two_directions(GripperFrame.BaseZ, axis)
            # rotate about z axis
            rotation_about_axis = np.array([[np.cos(rot), -np.sin(rot), 0],
                                            [np.sin(rot), np.cos(rot), 0],
                                            [0, 0, 1]])
        elif rot_axis_name == 'x':
            firstR_ax_an = pr.axis_angle_from_two_directions(GripperFrame.BaseX, axis)
            # rotate about x axis
            rotation_about_axis = np.array([[1, 0, 0],
                                            [0, np.cos(rot), -np.sin(rot)],
                                            [0, np.sin(rot), np.cos(rot)]])
        elif rot_axis_name == 'y':
            firstR_ax_an = pr.axis_angle_from_two_directions(GripperFrame.BaseY, axis)
            # rotate about x axis
            rotation_about_axis = np.array([[np.cos(rot), 0, np.sin(rot)],
                                            [0, 1, 0],
                                            [-np.sin(rot), 0, np.cos(rot)]])
        firstR = pr.matrix_from_axis_angle(firstR_ax_an)
        firstT = pt.transform_from(firstR, center)
        
        rotT = pt.transform_from(rotation_about_axis, np.array([0, 0, 0]))

        fullT = firstT @ rotT
        base = GripperFrame.base_frame()
        base.transform(fullT)

        return base

    def recover_in_plane_rot_angle(self):
        firstR_ax_an = pr.axis_angle_from_two_directions(GripperFrame.BaseZ, self.z)
        firstR = pr.matrix_from_axis_angle(firstR_ax_an)
        firstT = pt.transform_from(firstR, self.center)
        fullT = np.zeros((4, 4))
        fullT[:3, 0] = self._x
        fullT[:3, 1] = self._y
        fullT[:3, 2] = self._z
        fullT[:3, 3] = self.center
        fullT[3, 3] = 1.

        rotT = np.linalg.inv(firstT) @ fullT
        angle = np.arccos(rotT[0, 0])

        return angle

    @staticmethod
    def _get_R_aligning_vectors(v1, v2):
        ag_ax = pr.axis_angle_from_two_directions(v1, v2)  # 轴角表示
        return pr.matrix_from_axis_angle(ag_ax)

    def to_open3d_mesh(self):
        point_x = (self.x * 0.03 + self.center).tolist()
        point_y = (self.y * 0.03 + self.center).tolist()
        point_z = (self.z * 0.02 + self.center).tolist()
        points = [self.center.tolist(), point_x, point_y, point_z]
        lines = [[0, 1], [0, 2], [0, 3]]
        colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines))
        line_set.colors = o3d.utility.Vector3dVector(colors)

        return line_set

    def transform(self, T):
        """
        对整个坐标轴施加变换T
        :param T: 4x4的齐次变换矩阵
        :return:
        """
        # todo some failed here
        self._z = pt.transform(T, pt.vector_to_direction(self.z), strict_check=False)[:-1]
        self._y = pt.transform(T, pt.vector_to_direction(self.y), strict_check=False)[:-1]
        self._x = pt.transform(T, pt.vector_to_direction(self.x), strict_check=False)[:-1]
        self.center = pt.transform(T, pt.vector_to_point(self.center), strict_check=False)[:-1]

    def to_6dpose(self):
        rotation = np.array([self.x, self.y, self.z]).T
        return pt.transform_from(rotation, self.center)

    @staticmethod
    def from_6dpose(pose):
        center = pose[:3, 3]
        x = pose[:3, 0]
        y = pose[:3, 1]
        z = pose[:3, 2]
        return GripperFrame(x, y, z, center)

    def __str__(self):
        return f'Center={self.center}, dx={self.x}, dy={self.y}, dz={self.z}'

    def __repr__(self):
        return self.__str__()

    def __eq__(self, other):
        return check_equal(self.x, other.x) and check_equal(self.y, other.y) and check_equal(self.z, other.z) and check_equal(self.center, other.center)

