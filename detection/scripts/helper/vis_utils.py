#! /usr/bin/env python
# coding=utf-8

"""
抓取位姿在rviz可视化的一些帮助函数
"""
import rospy
from visualization_msgs.msg import Marker, MarkerArray

import pytransform3d.rotations as pyrot
import numpy as np


def generate_marker(pos, quat, scale, color, frame_id, ns, id, lifetime):
    """创建出一个长方体

    Args:
        pos (np.ndarray): 位置
        quat (np.ndarray): 姿态
        scale (np.ndarray): 三个维度的大小
        color (np.ndarray): 颜色, [0~1]
        frame_id (np.ndarray): 名字
        lifetime (np.ndarray): 持续时间

    Returns:
        visualization_msgs.msg.Marker: 长方体对象 
    """
    marker = Marker()
    # 指定这个marker的位置和姿态是基于哪个坐标系下的
    marker.header.frame_id = frame_id
    marker.header.stamp = rospy.Time.now()
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.ns = ns
    marker.id = id

    marker.pose.position.x = pos[0]
    marker.pose.position.y = pos[1]
    marker.pose.position.z = pos[2]
    marker.pose.orientation.x = quat[1]
    marker.pose.orientation.y = quat[2]
    marker.pose.orientation.z = quat[3]
    marker.pose.orientation.w = quat[0]

    marker.lifetime = rospy.Duration.from_sec(lifetime)
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    # alpha channel for color
    marker.color.a = 0.5
    r, g, b, a = color
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.color.a = a

    return marker

# 下面的参数都是为了可视化，而不是真实的夹爪数据
# 夹爪手指的高度
gripper_hand_height = 0.015
# 夹爪手指的宽度
gripper_finger_width = 0.02
# 夹爪最大张开快读
gripper_hand_outer_diameter = 0.14
# 夹爪手指的长度
gripper_hand_depth = 0.06
# gripper marker的命名空间
gripper_namespace = 'gripper_ns'

def generate_grasp_marker(pos, rotation, width, color, parent_frame, lifetime=3, rot_format='rot'):
    hh = gripper_hand_height
    fw = gripper_finger_width
    hod = width + 2 * gripper_finger_width
    hd = gripper_hand_depth
    open_w = width
    rotmat = rotation
    if rot_format == 'quat':
        assert rotation.shape == (4,), 'quaternion shape must be (4,) and form should be (w, x, y, z)'
        # 将quaternion转化为rotation matrix
        rotmat = pyrot.matrix_from_quaternion(rotation)

    assert rotmat.shape == (3, 3), 'rotation matrix shape must be (3, 3)'
    approach = rotmat[:, 0]
    binormal = rotmat[:, 1]
    minor_pc = rotmat[:, 2]
    grasp_bottom_center = pos - approach * gripper_hand_depth
    quat = pyrot.quaternion_from_matrix(rotmat)   # (w, x, y, z)
    
    bottom_pos = grasp_bottom_center - approach * hh * 0.5
    left_pos = grasp_bottom_center - binormal * (open_w * 0.5 + fw * 0.5) + hd * 0.5 * approach
    right_pos = grasp_bottom_center + binormal * (open_w * 0.5 + fw * 0.5) + hd * 0.5 * approach

    bottom_marker = generate_marker(bottom_pos, quat, np.array([hh, hod, hh]), 
                                    color, parent_frame, gripper_namespace, 0, lifetime)
    left_marker = generate_marker(left_pos, quat, np.array([hd, fw, hh]), 
                                    color, parent_frame, gripper_namespace, 1, lifetime)
    right_marker = generate_marker(right_pos, quat, np.array([hd, fw, hh]), 
                                    color, parent_frame, gripper_namespace, 2, lifetime)

    marker_array = MarkerArray()
    marker_array.markers.append(bottom_marker)
    marker_array.markers.append(left_marker)
    marker_array.markers.append(right_marker)

    return marker_array
