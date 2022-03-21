#! /usr/bin/env python
# coding=utf-8


"""
检测节点 - 负责使用点云进行抓取检测, 订阅realsense相机的点云信息
"""

from trimesh import PointCloud
import rospy
from sensor_msgs.msg import PointCloud2
from collections import deque
import open3d as o3d

from helper.pointcloud_utils import PointCloud2Utils
from inference_3d import SixDoFGraspDetector
from inference import CameraParams

P1 = '/home/ryan/Codes/pythonProject/grasping-repo/'
P2 = 'paper_grasp_project/6d_grasp/log/'
P3 = '220318-2258_graspness/ckpt/pointnet/'
P4 = 'epoch130_loss2.9654_acc0.920_prec0.910_rec0.863_rke0.172.tar'
HARD_CODED_MODEL_PATH = P1 + P2 + P3 + P4

# 一系列全局变量
NODE_DETECTION3D_NAME = "node_detection_3d"
# 存放点云 (np.ndarray)
_cloud_queue = deque(maxlen=2)


def subscribe_pointcloud2(cloud_msg):
    global _cloud_queue
    cloud = PointCloud2Utils.pointcloud2_to_xyz_array(cloud_msg)
    _cloud_queue.append(cloud)


def infer_3d():
    if len(_cloud_queue) != 0:
        raw_cloud = _cloud_queue.popleft()
        rospy.logdebug('raw_cloud shape = {}'.format(raw_cloud.shape))
        graspness_cloud = grasp_detector_3d.detect(raw_cloud)

        pointcloud2_msg = PointCloud2Utils.nparray_to_pointcloud2(graspness_cloud, 'camera_depth_optical_frame')

        dev_publisher.publish(pointcloud2_msg)
        rospy.loginfo('Published')


def before_shutdown():
    """
    节点结束前回调函数
    """
    rospy.loginfo("Shutting down detection_3d node...")


if __name__ == "__main__":
    rospy.init_node(name=NODE_DETECTION3D_NAME)
    rospy.on_shutdown(before_shutdown)

    # 订阅realsense的点云话题
    rospy.Subscriber("/camera/depth/color/points", PointCloud2, subscribe_pointcloud2, queue_size=3)

    # ! for development only, rename topic later
    dev_publisher = rospy.Publisher("/detection_3d/dev/graspness_cloud", PointCloud2, queue_size=5)

    # ! for development, hard coded model path here
    grasp_detector_3d = SixDoFGraspDetector(model_path=HARD_CODED_MODEL_PATH)
    
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        infer_3d()
        rate.sleep()
