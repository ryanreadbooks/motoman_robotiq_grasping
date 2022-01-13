#! /usr/bin/env python
# coding=utf-8

import os
import rospy
import subprocess
import roslaunch
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse

package = 'gripper_server'
executable_node = 'node_gripper_server.py'

launch_path = "/home/ryan/Codes/lab/yaskawa_vision_grasp/src/robotiq/gripper_server/launch/bringup_gripper_server.launch"
request_on = False

def service_handler(request: SetBoolRequest):
	global request_on
	rospy.loginfo("get start gripper_server request")
	request_on = request.data    # true表示继续检测，false表示停止检测
	rospy.sleep(5)
	response: SetBoolResponse = SetBoolResponse()

	response.message = 'Starting gripper_server'
	response.success = True # 设置成功
	print(__file__)
	path = os.path.join(os.path.dirname(__file__), 'start_gripper_server.sh')
	# os.system(path)
	subprocess.Popen(path, shell=True)
	rospy.loginfo('Starting gripper_server')

	return response

if __name__ == "__main__":
	gripper_server_on = False
	rospy.init_node('node_gripper_server_daemon')
	daemon_server = rospy.Service('node_gripper_daemon_service', SetBool, service_handler)
	rospy.loginfo('gripper daemon on...')

	rospy.spin()