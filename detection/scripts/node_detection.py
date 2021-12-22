#! /usr/bin/env python
# coding=utf-8


"""
检测节点：负责进行抓取检测，与realsense相机直接打交道
"""

from collections import deque
import os
import copy
import time
import rospy
import geometry_msgs.msg as gmsg
import tf2_ros
from cv_bridge import CvBridgeError, CvBridge
from sensor_msgs.msg import Image
import pytransform3d.rotations as pr

from inference import PlanarGraspDetector, CameraParams
# from helper.transform_utils import *


# 全局常量定义
NODE_DETECTION_NAME = 'node_detection'

# MODEL_PATH = '/home/ryan/Codes/pythonProject/grasping-repo/paper_grasp_project/light_detection/state_dicts/model.pth'



_cv_bridge: CvBridge = CvBridge()
# 队列用来存放realsense话题中的rgb图和depth图
_rgb_queue = deque(maxlen=2)
_depth_queue = deque(maxlen=2)

# tf广播器
# Create broadcast node
transform_br = tf2_ros.TransformBroadcaster()



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


def infer():
    """
    进行检测流程

    :param: color np.ndarray格式的RGB图片，shape=(H, W, 3)
    :param: depth np.ndarray格式的depth图片，shape=(H, W)
    """
    # 从队列中取数据
    if len(_rgb_queue) != 0 and len(_depth_queue) != 0:
        rgb_img = _cv_bridge.imgmsg_to_cv2(_rgb_queue.popleft())
        depth_img = _cv_bridge.imgmsg_to_cv2(_depth_queue.popleft())
        try :
            # realsense的深度需要处理成以m为单位
            start_time = time.time()
            res = grasp_detector.detect(rgb_img, depth_img / 1000.0)
            end_time = time.time()
            rospy.loginfo('get detection res from grasp detector, and took time %f seconds' % (end_time - start_time))
            if not res[0]:
                # 检测失败
                rospy.logwarn('Grasp detection failed, can not find grasps!')
            else:
                # 检测成功
                _, tcp_cam, rot_mat_cam, img_with_grasps = res
                # 发布TF
                stamped_transform = gmsg.TransformStamped()
                stamped_transform.header.stamp = rospy.Time.now()
                stamped_transform.header.frame_id = 'camera_link'
                stamped_transform.child_frame_id = 'grasp_candidate'
                stamped_transform.transform.translation.x = tcp_cam[0]
                stamped_transform.transform.translation.y = tcp_cam[1]
                stamped_transform.transform.translation.z = tcp_cam[2]
                # 旋转矩阵转化为四元数
                qw, qx, qy, qz = pr.quaternion_from_matrix(rot_mat_cam)
                stamped_transform.transform.rotation.x = qx
                stamped_transform.transform.rotation.y = qy
                stamped_transform.transform.rotation.z = qz
                stamped_transform.transform.rotation.w = qw

                transform_br.sendTransform(stamped_transform)

                result_img_publisher.publish(_cv_bridge.cv2_to_imgmsg(img_with_grasps[:,:,::-1]))   # 格式转换
                rospy.loginfo('Grasp detection succeed.')
        except Exception as e:
            rospy.logerr('Grasp detection failed due to exception {:s} of type {:s}'.format(str(e), str(type(e))))
    else:
        rospy.logwarn('Can not read images from realsense camera!!')

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
    result_img_publisher = rospy.Publisher('/ryan/detection/grasps_result_image', Image, queue_size=15)

    # 获取相机内参
    d435i = CameraParams(cx=rospy.get_param('~intrinsics/cx'), 
                         cy=rospy.get_param('~intrinsics/cy'),
                         fx=rospy.get_param('~intrinsics/fx'),
                         fy=rospy.get_param('~intrinsics/fy'),
                         fov=rospy.get_param('~intrinsics/fov'))

    grasp_detector = PlanarGraspDetector(model_path=rospy.get_param('~model_path'), camera_params=d435i)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        infer()
        rate.sleep()
