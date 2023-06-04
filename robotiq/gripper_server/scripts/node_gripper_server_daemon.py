#! /usr/bin/env python
# coding=utf-8

import os
import rospy
import subprocess
import rosnode
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

package = 'gripper_server'
executable_node = 'node_gripper_server.py'
launch_path = os.path.join(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))), 'launch/bringup_gripper_server.launch')
print(f'launch_path={launch_path}')
request_on = False


def service_handler(request: SetBoolRequest):
    global request_on
    rospy.loginfo("get start gripper_server request")
    request_on = request.data    # true表示继续检测，false表示停止检测
    rospy.sleep(2)
    response: SetBoolResponse = SetBoolResponse()

    response.message = 'Starting gripper_server'
    response.success = True  # 设置成功
    print(__file__)
    path = os.path.join(os.path.dirname(__file__), 'start_gripper_server.sh')
    # os.system(path)
    subprocess.Popen(path, shell=True)
    rospy.loginfo('Starting gripper_server')

    return response


def on_shutdown():
    rospy.loginfo(
        'Now shutting down gripper server daemon server, below is running nodes...')
    print(rosnode.get_node_names())


if __name__ == "__main__":
    gripper_server_on = False
    rospy.init_node('node_gripper_server_daemon')
    rospy.on_shutdown(on_shutdown)
    daemon_server = rospy.Service(
        'node_gripper_daemon_service', SetBool, service_handler)
    rospy.loginfo('gripper daemon on...')

    rospy.spin()
