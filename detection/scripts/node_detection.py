#! /usr/bin/env python
# coding=utf-8


"""
检测节点 - 负责进行抓取检测，与realsense相机直接打交道
"""

from collections import deque
import os
import copy
import time
import rospy
import geometry_msgs.msg as gmsg
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import tf2_ros
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
import pytransform3d.rotations as pr

from detection.msg import DetectionResult
from inference import PlanarGraspDetector, CameraParams
# from helper.transform_utils import *


# 全局常量定义
NODE_DETECTION_NAME = 'node_detection'


_cv_bridge: CvBridge = CvBridge()
# 队列用来存放realsense话题中的rgb图和depth图
_rgb_queue = deque(maxlen=2)
_depth_queue = deque(maxlen=2)

# 标志是否正在进行检测
_detection_on = True

# tf广播器
# Create broadcast node
tf_broadcaster = tf2_ros.TransformBroadcaster()


def subscribe_rgb(rgb: Image):
    """
    回调函数，将话题中的rgb图像放入全局双端队列中
    """
    global _rgb_queue
    _rgb_queue.append(rgb)


def subscribe_depth(depth: Image):
    """
    回调函数，将话题中的depth图像放入全局双端队列中
    """
    global _depth_queue
    _depth_queue.append(depth)


def switch_service_handler(request: SetBoolRequest, ):
    global _detection_on
    _detection_on = request.data    # true表示继续检测，false表示停止检测

    if _detection_on:
        rospy.loginfo('Detection is going on')
    else:
        rospy.loginfo('Detection paused')

    response: SetBoolResponse = SetBoolResponse()
    response.success = True # 设置成功
    response.message = 'Stop detecting' if not _detection_on else "Start detecting"

    return response


def infer():
    """
    进行检测流程

    :param: color np.ndarray格式的RGB图片，shape=(H, W, 3)
    :param: depth np.ndarray格式的depth图片，shape=(H, W)
    """
    # 从队列中取数据
    result_msg = DetectionResult()
    if len(_rgb_queue) != 0 and len(_depth_queue) != 0:
        rgb_img = _cv_bridge.imgmsg_to_cv2(_rgb_queue.popleft())
        depth_img = _cv_bridge.imgmsg_to_cv2(_depth_queue.popleft())
        try :
            # realsense的深度需要处理成以m为单位，在grasp_detector.detect函数内部会进行处理
            start_time = time.time()
            res = grasp_detector.detect(rgb_img, depth_img)
            end_time = time.time()
            rospy.loginfo('get detection res from grasp detector, and took time %f seconds' % (end_time - start_time))
            if not res[0]:
                # 检测失败
                result_msg.success = False
                result_msg.message = 'Grasp detection failed, can not find grasps!'
                rospy.logwarn('Grasp detection failed, can not find grasps!')
            else:
                # 检测成功
                _, tcp_cam, rot_mat_cam, img_with_grasps, physical_grasp_width = res
                # 发布TF
                # TODO pose好像不太对
                stamped_transform = gmsg.TransformStamped()
                stamped_transform.header.stamp = rospy.Time.now()
                # 发布相机坐标系下的抓取姿态坐标，这里如果用camera_link的话，不work
                stamped_transform.header.frame_id = 'camera_depth_optical_frame'
                stamped_transform.child_frame_id = 'grasp_candidate'
                stamped_transform.transform.translation.x = tcp_cam[0]
                stamped_transform.transform.translation.y = tcp_cam[1]
                stamped_transform.transform.translation.z = tcp_cam[2]
                # 旋转矩阵转化为四元数
                qw, qx, qy, qz = pr.quaternion_from_matrix(rot_mat_cam)
                rospy.loginfo('qw={:.5f}, qx={:.5f}, qy={:.5f}, qz={:.5f}'.format(qw, qx, qy, qz))
                stamped_transform.transform.rotation.x = qx
                stamped_transform.transform.rotation.y = qy
                stamped_transform.transform.rotation.z = qz
                stamped_transform.transform.rotation.w = qw

                tf_broadcaster.sendTransform(stamped_transform)
                # 将检测的结果图片发布出去
                result_img_publisher.publish(_cv_bridge.cv2_to_imgmsg(img_with_grasps[:,:,::-1]))   # 格式转换

                result_msg.success = True
                result_msg.grasp_width = physical_grasp_width
                result_msg.transformation = stamped_transform
                result_msg.message = 'Success'
                rospy.loginfo('Grasp detection succeed.')
        except Exception as e:
            rospy.logerr('Grasp detection failed due to exception {:s}, exception type is {:s}'.format(str(e), str(type(e))))
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
    rospy.Subscriber('/camera/color/image_raw', Image, subscribe_rgb, queue_size=3)
    rospy.Subscriber('/camera/aligned_depth_to_color/image_raw', Image, subscribe_depth, queue_size=3)

    # 发布检测结果的图片的话题
    result_img_publisher = rospy.Publisher('/detection/grasps_result_image', Image, queue_size=5)
    # 检测结果的发布话题
    detection_res_publisher = rospy.Publisher('/detection/result', DetectionResult, queue_size=5)

    # 用一个服务来接收是否持续进行检测
    detection_server = rospy.Service('/detection/switch_service', SetBool, switch_service_handler)

    # 获取相机内参
    d435i = CameraParams(cx=rospy.get_param('~intrinsics/cx'), 
                         cy=rospy.get_param('~intrinsics/cy'),
                         fx=rospy.get_param('~intrinsics/fx'),
                         fy=rospy.get_param('~intrinsics/fy'),
                         fov=rospy.get_param('~intrinsics/fov'))

    grasp_detector = PlanarGraspDetector(model_path=rospy.get_param('~model_path'), camera_params=d435i)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if _detection_on:
            infer()
            rate.sleep()
