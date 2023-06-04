#! /usr/bin/env python
# coding=utf-8


"""
检测节点 - 负责进行抓取检测,与realsense相机直接打交道
"""

import sys
import time
from collections import deque
import copy

import rospy
import geometry_msgs.msg as gmsg
import tf2_ros
import tf.transformations as tft
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
from visualization_msgs.msg import MarkerArray

import numpy as np
import pytransform3d.rotations as pr

from helper.transform_utils import convert_pose, publish_pose_as_transform, publish_grasp_candidate_using_6d_pose
from detection.msg import DetectionResult
from inference import PlanarGraspDetector, CameraParams
from helper.node_utils import subscribe_rgb, subscribe_depth, _depth_queue, _rgb_queue, _cv_bridge, d435i
from helper.vis_utils import generate_grasp_marker
from helper.constants import *
from helper.common_utils import get_exception_trackback


# 全局常量定义
NODE_DETECTION_NAME = 'node_detection'
_detection_on = True

spatial = False
compensated = True


def get_rotmat_around_y(ry):
    """
    获取绕y轴旋转的旋转矩阵, 在zx平面逆时针
    :param ry: 旋转角度,单位rad
    :return:
    """
    return np.array([[np.cos(ry), 0, np.sin(ry)],
                     [0, 1, 0],
                     [-np.sin(ry), 0, np.cos(ry)]])


def switch_service_handler(request: SetBoolRequest):
    global _detection_on
    _detection_on = request.data

    if _detection_on:
        rospy.loginfo('Detection is going on...')
    else:
        rospy.loginfo('Detection paused.')

    response: SetBoolResponse = SetBoolResponse()
    response.success = True  # 设置成功
    response.message = 'Stop detecting' if not _detection_on else "Start detecting"

    return response


def infer():
    """
    进行检测流程

    :param: color np.ndarray格式的RGB图片,shape=(H, W, 3)
    :param: depth np.ndarray格式的depth图片,shape=(H, W)
    """
    # 从队列中取数据
    if spatial or compensated:
        method = DetectionResult.METHOD_SPATIAL
    else:
        method = DetectionResult.METHOD_PLANAR
    result_msg = DetectionResult(method=method)
    if len(_rgb_queue) != 0 and len(_depth_queue) != 0:
        rgb_img = _cv_bridge.imgmsg_to_cv2(_rgb_queue.popleft())
        depth_img = _cv_bridge.imgmsg_to_cv2(_depth_queue.popleft())
        try:
            # realsense的深度需要处理成以m为单位，在grasp_detector.detect函数内部会进行处理
            start_time = time.time()
            res = grasp_detector.detect(rgb_img, depth_img)
            end_time = time.time()
            rospy.loginfo('get detection res from grasp detector, and took time %f seconds' % (
                end_time - start_time))
            if not res[0]:
                # 检测失败
                result_msg.success = False
                result_msg.message = 'Grasp detection failed, can not find grasps!'
                rospy.logwarn('Grasp detection failed, can not find grasps!')
            else:
                # 检测成功
                _, ret_map = res
                tcp_cam = ret_map[PlanarGraspDetector.KeyGraspCenter]
                angle = ret_map[PlanarGraspDetector.KeyAngle]
                rot_mat_cam = ret_map[PlanarGraspDetector.KeyRotMatCam]
                img_with_grasps = ret_map[PlanarGraspDetector.KeyImgWithGrasp]
                physical_grasp_width = ret_map[PlanarGraspDetector.KeyPhysicalWidth]
                img_original_with_p = ret_map[PlanarGraspDetector.KeyImgOriginalWithP]
                graspness_on_img = ret_map[PlanarGraspDetector.KeyImgGraspness]
                rotmat_wrt_camera = ret_map[PlanarGraspDetector.KeyRotMat]
                img_original_cropped = ret_map[PlanarGraspDetector.KeyImgCropped]

                # 检测的姿态结果发布出去,相机坐标系和抓取坐标系之间的变换关系

                # 旋转矩阵转化为四元数
                qw, qx, qy, qz = pr.quaternion_from_matrix(rot_mat_cam)
                rospy.loginfo('qw={:.5f}, qx={:.5f}, qy={:.5f}, qz={:.5f}, width={:.3f}'.format(
                    qw, qx, qy, qz, physical_grasp_width))

                # 构建相机坐标系下的姿态
                gp = gmsg.Pose()
                gp.position.x = tcp_cam[0]
                gp.position.y = tcp_cam[1]
                gp.position.z = tcp_cam[2]
                gp.orientation.w = 1

                # 将检测的结果图片发布出去
                result_img_publisher.publish(_cv_bridge.cv2_to_imgmsg(
                    img_with_grasps[:, :, ::-1]))   # 格式转换
                result_img_publisher_ori.publish(_cv_bridge.cv2_to_imgmsg(
                    img_original_with_p[:, :, ::-1]))   # 格式转换
                result_img_publisher_gsp.publish(
                    _cv_bridge.cv2_to_imgmsg(graspness_on_img[:, :, ::-1]))   # 格式转换
                result_img_publisher_ori_cropped.publish(
                    _cv_bridge.cv2_to_imgmsg(img_original_cropped[:, :, ::-1]))

                # 转换到基座标系下，三维平移先变换过去
                gp_base = convert_pose(
                    gp, 'camera_color_optical_frame', 'base_link')
                if gp_base is None:
                    raise RuntimeError(
                        'Can not calculate converted pose, may be robot not connected')
                # 再单独变换姿态，角度全部统一成顺时针旋转

                q = tft.quaternion_from_euler(
                    np.pi + angle, np.pi / 2,  0)

                gp_base_orientation_mat = pr.matrix_from_quaternion(
                    np.array([q[3], q[0], q[1], q[2]]))

                if compensated:
                    compensated_angle = 0
                    rotmat_compensation = get_rotmat_around_y(
                        np.deg2rad(compensated_angle))
                else:
                    rotmat_compensation = np.array([[1.0, 0.0, 0.0],
                                                    [0.0, 1.0, 0.0],
                                                    [0.0, 0.0, 1.0]])

                rotmat_base = gp_base_orientation_mat @ rotmat_compensation
                q_com = pr.quaternion_from_matrix(rotmat_base)

                gp_base.orientation.x = q_com[1]
                gp_base.orientation.y = q_com[2]
                gp_base.orientation.z = q_com[3]
                gp_base.orientation.w = q_com[0]

                forward_placement = rotmat_base[:, 0] * 0.018  # 前进
                gp_base.position.x += forward_placement[0]
                gp_base.position.y += forward_placement[1]
                gp_base.position.z += forward_placement[2]

                # base_link和grasp_candidate之间的转换关系通过TF发布
                # 这个gp_base就是在基坐标系下的抓取姿态
                publish_pose_as_transform(
                    gp_base, "base_link", "grasp_candidate", 0)
                rospy.loginfo("grasp_candidate tf published")

                if compensated:
                    # 发布预抓取点的位姿TF
                    rospy.loginfo("compensated calculating pre_grasp_pose")
                    # 沿着抓取反向30cm
                    placement = rotmat_base[:, 0] * -0.30
                    pre_gp_base = copy.deepcopy(gp_base)
                    pre_gp_base.position.x += placement[0]
                    pre_gp_base.position.y += placement[1]
                    pre_gp_base.position.z += placement[2]
                    pre_gp_base.orientation = gp_base.orientation

                    publish_pose_as_transform(
                        pre_gp_base, "base_link", "pre_grasp_pose", 0)

                    # 预抓取位姿的可视化
                    pre_grasp_vis_model = generate_grasp_marker(pos=np.array([pre_gp_base.position.x,
                                                                             pre_gp_base.position.y,
                                                                             pre_gp_base.position.z]),
                                                                rotation=np.array(
                                                                    [q[3], q[0], q[1], q[2]]),
                                                                width=physical_grasp_width,
                                                                color=np.array(
                                                                    [0.0, 1.0, 0.8, 0.9]),
                                                                parent_frame='base_link', rot_format='quat')

                    pre_grasp_vis_publisher.publish(pre_grasp_vis_model)

                    rospy.loginfo("pre_grasp_pose tf published")

                publish_grasp_candidate_using_6d_pose(
                    rotmat_wrt_camera, tcp_cam, 'grasp_candidate-debug')

                # 夹爪位姿的可视化，相对于base_link
                pos = np.array(
                    [gp_base.position.x, gp_base.position.y, gp_base.position.z])
                rot = np.array([gp_base.orientation.w, gp_base.orientation.x,
                               gp_base.orientation.y, gp_base.orientation.z])
                grasp_vis_model = generate_grasp_marker(pos=pos, rotation=rot,
                                                        width=physical_grasp_width,
                                                        color=np.array(
                                                            [0.0, 1.0, 0.0, 0.9]),
                                                        parent_frame='base_link', rot_format='quat')

                grasp_vis_publisher.publish(grasp_vis_model)

                result_msg.angle = angle
                result_msg.grasp_width = physical_grasp_width
                result_msg.message = 'Success'
                result_msg.success = True
                rospy.loginfo('Grasp detection succeed.')
        except Exception as e:
            result_msg.success = False
            result_msg.message = 'Failed[{:s}]'.format(str(e))
            rospy.logerr(get_exception_trackback(e))
            rospy.logerr('Grasp detection failed due to exception {:s}, exception type is {:s}'.format(
                str(e), str(type(e))))
    else:
        result_msg.success = False
        result_msg.message = 'Can not read images from realsense camera!!'
        rospy.logwarn('Can not read images from realsense camera!!')
    detection_res_publisher.publish(result_msg)


def before_shutdown():
    """
    节点结束前回调函数
    """
    rospy.loginfo('Shutting down detection node...')


if __name__ == '__main__':
    rospy.init_node(name=NODE_DETECTION_NAME)
    rospy.on_shutdown(before_shutdown)

    # 订阅realsense的话题，订阅RGB图和深度图
    rospy.Subscriber(REALSENSE_COLOR_TOPIC, Image, subscribe_rgb, queue_size=3)
    rospy.Subscriber(REALSENSE_DEPTH_TOPIC, Image,
                     subscribe_depth, queue_size=3)

    queue_size = 5

    NODE_TOPIC_PREFIX = '/detection/'
    # 发布检测结果的图片的话题
    result_img_publisher = rospy.Publisher(
        NODE_TOPIC_PREFIX + 'grasps_result_image', Image, queue_size=queue_size)
    result_img_publisher_ori = rospy.Publisher(
        NODE_TOPIC_PREFIX + 'grasps_result_image_ori', Image, queue_size=queue_size)
    result_img_publisher_gsp = rospy.Publisher(
        NODE_TOPIC_PREFIX + 'grasps_result_image_graspness', Image, queue_size=queue_size)
    result_img_publisher_ori_cropped = rospy.Publisher(
        NODE_TOPIC_PREFIX + 'grasps_result_image_ori_cropped', Image, queue_size=queue_size)

    compensated = rospy.get_param('~compensated', True)
    spatial = rospy.get_param('~spatial', False)

    # 检测结果的发布话题
    if spatial or compensated:
        NODE_TOPIC_PREFIX_3D = '/detection_3d/'
    else:
        NODE_TOPIC_PREFIX_3D = NODE_TOPIC_PREFIX
    detection_res_publisher = rospy.Publisher(
        NODE_TOPIC_PREFIX_3D + 'result', DetectionResult, queue_size=5)

    # 用一个服务来接收是否持续进行检测
    detection_server = rospy.Service(
        NODE_TOPIC_PREFIX + 'switch_service', SetBool, switch_service_handler)

    # 发布可视化的结果
    if not compensated:
        vis_topic_name = NODE_TOPIC_PREFIX + 'grasp_vis'
        pre_vis_topic_name = NODE_TOPIC_PREFIX + 'pre_grasp_vis'
    else:
        vis_topic_name = NODE_TOPIC_PREFIX_3D + 'grasp_vis'
        pre_vis_topic_name = NODE_TOPIC_PREFIX_3D + 'pre_grasp_vis'

    grasp_vis_publisher = rospy.Publisher(
        vis_topic_name, MarkerArray, queue_size=1)
    pre_grasp_vis_publisher = rospy.Publisher(
        pre_vis_topic_name, MarkerArray, queue_size=1)

    grasp_detector = PlanarGraspDetector(model_path=rospy.get_param('~model_path'), camera_params=d435i,
                                         normal_as_direction=True,
                                         threshold=0.7,
                                         use_rgbd=rospy.get_param("~use_rgbd"))

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        if _detection_on:
            if not spatial:
                infer()
            rate.sleep()
