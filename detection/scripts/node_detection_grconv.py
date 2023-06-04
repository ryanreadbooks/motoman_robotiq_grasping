#! /usr/bin/env python
# coding=utf-8

import sys
import os
import time
import numpy as np

import rospy
import geometry_msgs.msg as gmsg
import tf2_ros
import tf.transformations as tft
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

from detection.msg import DetectionResult
from helper.node_utils import subscribe_rgb, subscribe_depth, _depth_queue, _rgb_queue, _cv_bridge, d435i
from inference_grconv import GraspGenerator
from helper.common_utils import get_exception_trackback
from helper.vis_utils import generate_grasp_marker
from helper.transform_utils import convert_pose, publish_pose_as_transform, publish_grasp_candidate_using_6d_pose
from helper.constants import *


# 全局常量定义
NODE_DETECTION_NAME = 'node_detection_grconv'
_detection_on = True


def switch_service_handler(request: SetBoolRequest):
    global _detection_on
    _detection_on = request.data

    if _detection_on:
        rospy.loginfo('Detection-GRConv is going on...')
    else:
        rospy.loginfo('Detection-GRConv paused.')

    response: SetBoolResponse = SetBoolResponse()
    response.success = True  # 设置成功
    response.message = 'Stop detecting' if not _detection_on else "Start detecting"

    return response


def infer():
	result_msg = DetectionResult(method=DetectionResult.METHOD_PLANAR)
	if len(_rgb_queue) != 0 and len(_depth_queue) != 0:
		rgb_img = _cv_bridge.imgmsg_to_cv2(_rgb_queue.popleft())
		depth_img = _cv_bridge.imgmsg_to_cv2(_depth_queue.popleft())
		try:
			res = grasp_detector.detect(rgb_img, depth_img)
			tcp_cam, rotmat, angle, phy_width, rbg_img_vis, result_img = res
			
			# 生成抓取姿态
			gp = gmsg.Pose()
			gp.position.x = tcp_cam[0]
			gp.position.y = tcp_cam[1]
			gp.position.z = tcp_cam[2]
			gp.orientation.w = 1

			# 转换到基座标系下，三维平移先变换过去
			gp_base = convert_pose(gp, 'camera_color_optical_frame', 'base_link')
			if gp_base is None:
				raise RuntimeError('Can not calculate converted pose, may be robot not connected')
			# 再单独变换姿态，角度全部统一成顺时针旋转
			q = tft.quaternion_from_euler(-angle, np.pi / 2, 0)
			gp_base.orientation.x = q[0]
			gp_base.orientation.y = q[1]
			gp_base.orientation.z = q[2]
			gp_base.orientation.w = q[3]
			# base_link和grasp_candidate之间的转换关系通过TF发布
			# 这个gp_base就是在基坐标系下的抓取姿态
			publish_pose_as_transform(gp_base, "base_link", "grasp_candidate", 0)

			# 夹爪位姿的可视化，相对于base_link
			pos = np.array([gp_base.position.x, gp_base.position.y, gp_base.position.z])
			rot = np.array([gp_base.orientation.w, gp_base.orientation.x, gp_base.orientation.y, gp_base.orientation.z])
			grasp_vis_model = generate_grasp_marker(pos=pos, rotation=rot, 
													width=phy_width,
													color=np.array([0.0, 1.0, 0.0, 0.9]), 
													parent_frame='base_link', rot_format='quat')

			grasp_vis_publisher.publish(grasp_vis_model)

			# 将检测的结果图片发布出去
			result_img_publisher.publish(_cv_bridge.cv2_to_imgmsg(result_img[:,:,::-1]))   # 格式转换
			result_img_publisher_ori.publish(_cv_bridge.cv2_to_imgmsg(rbg_img_vis[:,:,::-1]))   # 格式转换

			rospy.loginfo(f'tcp={tcp_cam}, angle={-angle}, phy_width={phy_width}')

			result_msg.angle = angle
			result_msg.grasp_width = phy_width
			result_msg.message = 'Success'
			result_msg.success = True
			rospy.loginfo('Grasp detection succeed.')
		except Exception as e:
			rospy.logerr(get_exception_trackback(e))
			rospy.logerr(f'Grasp detection failed due to: {str(e)}')
			result_msg.success = False
			result_msg.message = 'Failed[{:s}]'.format(str(e))
	else:
		result_msg.success = False
		result_msg.message = 'Can not read images from realsense camera!!'
		rospy.logwarn('Can not read images from realsense camera!!')
	detection_res_publisher.publish(result_msg)


if __name__ == '__main__':
	rospy.init_node(name=NODE_DETECTION_NAME)

	# 订阅realsense的话题，订阅RGB图和深度图
	rospy.Subscriber(REALSENSE_COLOR_TOPIC, Image, subscribe_rgb, queue_size=3)
	rospy.Subscriber(REALSENSE_DEPTH_TOPIC, Image, subscribe_depth, queue_size=3)

	NODE_TOPIC_PREFIX = '/detection/'
	# 发布检测结果的图片的话题
	result_img_publisher = rospy.Publisher(
    NODE_TOPIC_PREFIX + 'grasps_result_image', Image, queue_size=5)
	result_img_publisher_ori = rospy.Publisher(
    NODE_TOPIC_PREFIX + 'grasps_result_image_ori', Image, queue_size=5)
	result_img_publisher_gsp = rospy.Publisher(
    NODE_TOPIC_PREFIX + 'grasps_result_image_graspness', Image, queue_size=5)

	# 检测结果的发布话题
	NODE_TOPIC_PREFIX_3D = '/detection_3d/'
	detection_res_publisher = rospy.Publisher(NODE_TOPIC_PREFIX_3D + 'result', DetectionResult, queue_size=5)

	# 用一个服务来接收是否持续进行检测
	detection_server = rospy.Service(NODE_TOPIC_PREFIX + 'switch_service', SetBool, switch_service_handler)

		# 发布可视化的结果
	grasp_vis_publisher = rospy.Publisher(NODE_TOPIC_PREFIX + 'grasp_vis', MarkerArray, queue_size=1)
	grasp_detector = GraspGenerator(camera_params=d435i)

	rate = rospy.Rate(10) # 10hz
	while not rospy.is_shutdown():
		if _detection_on:
			infer()
			rate.sleep()
