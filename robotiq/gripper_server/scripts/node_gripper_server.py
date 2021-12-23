#! /usr/bin/env python
# coding=utf-8


from os import read
import rospy
# Brings in the SimpleActionClient
import actionlib

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from gripper_server.srv import GripperServiceRequest, GripperServiceResponse, GripperService
from gripper_core import GripperServer

gripper_server = None


def request_handler(req: GripperServiceRequest):
    """
    请求处理函数
    """
    rospy.loginfo(f'Get request {req}')

    # 组织响应值
    response = gripper_server.handle_request(req)

    return response


if __name__ == '__main__':
    rospy.init_node('node_gripper_server')
    rospy.loginfo('Connecting to robotiq gripper')

    action_name = rospy.get_param('~action_name', 'command_robotiq_action')
    _robotiq_client = actionlib.SimpleActionClient(action_name, CommandRobotiqGripperAction)
    # wait until grippers are ready to take command
    rospy.loginfo('Waiting for robotiq server to response ... ')
    ready = _robotiq_client.wait_for_server()
    if ready:
        rospy.loginfo('Robotiq action server is ready to take commands')
    else:
        rospy.logerr('Robotiq gripper can not take command, now exitting... ')
        exit(-1)

    gripper_server = GripperServer(_robotiq_client)
    rospy.loginfo('Connected to robotiq gripper.')
    ros_server = rospy.Service('node_gripper_server', GripperService, request_handler)
    rospy.loginfo('Gripper server on and is ready to receive requests...')

    rospy.spin()
