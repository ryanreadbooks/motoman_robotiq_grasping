#! /usr/bin/env python
# coding=utf-8

"""
检测节点帮助函数 - 定义一些检测节点都能用到的公共函数
"""
from collections import deque

import rospy
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image

from inference import CameraParams

_cv_bridge = CvBridge()
# 队列用来存放realsense话题中的rgb图和depth图
_rgb_queue = deque(maxlen=5)
_depth_queue = deque(maxlen=5)
# 标记是否处于检测状态
_detection_on = True
# realsense d435i相机内参
d435i = CameraParams(cx=rospy.get_param('/detection/intrinsics/cx'),
                     cy=rospy.get_param('/detection/intrinsics/cy'),
                     fx=rospy.get_param('/detection/intrinsics/fx'),
                     fy=rospy.get_param('/detection/intrinsics/fy'),
                     fov=rospy.get_param('/detection/intrinsics/fov'))


def subscribe_rgb(rgb: Image):
    """
    回调函数,将话题中的rgb图像放入全局双端队列中
    """
    global _rgb_queue
    _rgb_queue.append(rgb)


def subscribe_depth(depth: Image):
    """
    回调函数,将话题中的depth图像放入全局双端队列中
    """
    global _depth_queue
    _depth_queue.append(depth)
