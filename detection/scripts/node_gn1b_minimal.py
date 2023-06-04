#! /usr/bin/env python
# coding=utf-8

"""
检测节点 - 负责使用GN1B-Minimal进行抓取检测, 订阅realsense相机的点云信息
"""

import copy
import numpy as np
from collections import deque
import time
import cv2

import rospy
import tf2_ros
import geometry_msgs.msg as gmsg
from sensor_msgs.msg import PointCloud2
import tf.transformations as tft
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

from helper.common_utils import get_traceback
from helper.transform_utils import publish_grasp_candidate_using_6d_pose
from gn1b_minimal import GN1BMinimal 
from detection.msg import DetectionResult
from helper.node_utils import subscribe_rgb, subscribe_depth, _depth_queue, _rgb_queue, _cv_bridge, d435i
from helper.vis_utils import generate_grasp_marker
from helper.constants import *


NODE_DETECTION_GN1B_NAME = "node_detection_gn1b_minimal"
_detection_on = True


def switch_service_handler(request: SetBoolRequest):
    global _detection_on
    _detection_on = request.data

    if _detection_on:
        rospy.loginfo('Detection_3d is going on...')
    else:
        rospy.loginfo('Detection_3d paused.')

    response: SetBoolResponse = SetBoolResponse()
    response.success = True  # 设置成功
    response.message = 'Stop detecting' if not _detection_on else "Start detecting"

    return response


def infer_with_gn1b_minimal():
    # 从队列中取数据
    result_msg = DetectionResult(method=DetectionResult.METHOD_SPATIAL)
    begin_time = time.time()
    # if len(_cloud_queue) != 0:
    if len(_depth_queue) != 0 and len(_rgb_queue) != 0:
        try:
            rgb_img = _cv_bridge.imgmsg_to_cv2(_rgb_queue.popleft()) / 255.0
            rgb_img = rgb_img[:,:,::-1]
            depth_img = _cv_bridge.imgmsg_to_cv2(_depth_queue.popleft())   # 这里的单位不是m,但是在create_point_cloud_from_depth_image函数内进行了转换

            grasp_rot, grasp_trans, grasp_center, grasp_width, grasp_score, pre_grasp_point, approaching, angle = detector.detect(rgb_img, depth_img)
            rospy.loginfo(f'Grasp score = {grasp_score}, width = {grasp_width}')
            # 发布抓取位姿TF, 相对于camera_color_optical_frame坐标系
            publish_grasp_candidate_using_6d_pose(grasp_rot, grasp_center, 'grasp_candidate')
            # 发布预抓取点的位姿TF, 相对于camera_color_optical_frame坐标系
            publish_grasp_candidate_using_6d_pose(grasp_rot, pre_grasp_point, 'pre_grasp_pose')

            # 发布marker作为grasp pose, 其位姿是相对与camera_color_optical_frame
            # color = np.array([1 - grasp_score, grasp_score, 0., 0.9])   # rgba, make color according to score
            color = np.array([0.0, 1.0, 0.0, 0.9])   # rgba, make gripper green
            grasp_vis_model = generate_grasp_marker(pos=grasp_center, rotation=grasp_rot,
                                                    width=grasp_width, color=color, 
                                                    parent_frame='camera_color_optical_frame')
            grasp_vis_publisher.publish(grasp_vis_model)

            # 发布检测结果
            result_msg.pre_grasp_point = gmsg.Vector3(x=pre_grasp_point[0], y=pre_grasp_point[1], z=pre_grasp_point[2])
            result_msg.angle = angle
            result_msg.grasp_width = grasp_width
            result_msg.success = True
            result_msg.message = 'Success'
            rospy.loginfo('GN1B-Minimal grasp detection succeed')
        except Exception as ex:
            result_msg.success = False
            result_msg.message = 'Failed[{:s}]'.format(str(ex))
            # 打印异常堆栈
            rospy.logerr(get_traceback(ex.__traceback__))
            rospy.logerr('GN1B-Minimal grasp detection failed. [Exception=\'{:s}\', type={:s}'.format( str(ex), str(type(ex))))
    else:
        result_msg.success = False
        result_msg.message = 'Can not read pointcloud from realsense camera!!'
        rospy.logwarn('Can not read pointcloud from realsense camera!!')
    detection_gn1b_res_publisher.publish(result_msg)
    end_time = time.time()
    rospy.loginfo(f'Finish GN1B-Minimal once took {end_time - begin_time} s')


if __name__ == "__main__":
    rospy.init_node(name=NODE_DETECTION_GN1B_NAME)

    # 订阅realsense的话题，订阅RGB图和深度图
    rospy.Subscriber(REALSENSE_COLOR_TOPIC, Image,
                     subscribe_rgb, queue_size=3)
    rospy.Subscriber(REALSENSE_DEPTH_TOPIC, Image, 
                     subscribe_depth, queue_size=3)

    # 检测启停服务
    NODE3D_TOPIC_PREFIX = '/detection_3d/'
    detection_gn1b_server = rospy.Service(
        NODE3D_TOPIC_PREFIX + 'switch_service', SetBool, switch_service_handler)
    # 检测结果的发布话题
    detection_gn1b_res_publisher = rospy.Publisher(
        NODE3D_TOPIC_PREFIX + 'result', DetectionResult, queue_size=5)

    # 发布可视化的结果
    grasp_vis_publisher = rospy.Publisher(NODE3D_TOPIC_PREFIX + 'grasp_vis', MarkerArray, queue_size=1)

    detector = GN1BMinimal(cam_param=d435i)

    rate = rospy.Rate(0.5)  # 10hz
    while not rospy.is_shutdown():
        if _detection_on:
            infer_with_gn1b_minimal()
        rate.sleep()
