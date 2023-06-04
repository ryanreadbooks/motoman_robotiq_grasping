from os import stat
from time import time
import json

import rospy
import actionlib
from std_srvs.srv import SetBool, SetBoolRequest

from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperActionFeedback
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import RobotiqGripperStatus

from .req_res_code import *
from gripper_server.srv import GripperServiceRequest, GripperServiceResponse, GripperService


def result_to_json_str(result: CommandRobotiqGripperResult) -> str:
    header = result.header
    is_ready = result.is_ready
    is_reset = result.is_reset
    is_moving = result.is_moving
    obj_detected = result.obj_detected
    fault_status = result.fault_status
    position = result.position
    requested_position = result.requested_position
    current = result.current

    d = {
            'header': {'seq': header.seq, 
                        'stamp': {'secs' : header.stamp.secs, 
                                    'nsecs': header.stamp.nsecs
                                }
                    },
            'is_ready': is_ready,
            'is_reset': is_reset,
            'is_moving': is_moving,
            'obj_detected': obj_detected,
            'fault_status': fault_status,
            'position': position,
            'requested_position': requested_position,
            'current': current
        }

    return json.dumps(d)


class GripperServer:

    OK = 200
    FAIL = 400

    def __init__(self, robotiq_client) -> None:
        """
        初始化
        """
        print("init GripperServer")
        # 包含一个夹爪控制对象
        self._robotiq_client: actionlib.SimpleActionClient = robotiq_client
        rospy.Subscriber('/robotiq/gripper_online_status', RobotiqGripperStatus, self.online_status_cb, queue_size=10)
        self._gripper_status = None # 缓存最近一次的夹爪状态，主要感兴趣的是obj_detected这个标志

    def online_status_cb(self, status: RobotiqGripperStatus):
        self._gripper_status = status

    def handle_request(self, request: GripperServiceRequest):
        """
        处理请求
        """
        reqcode = request.reqcode
        response = GripperServiceResponse()
        response.reqcode = reqcode
        if reqcode == ReqGripperManualAction:
            rescode, resdata = self._manual_action(request.position, request.speed, request.force)
        elif reqcode == ReqGripperStop:
            rescode, resdata = self._stop()
        elif reqcode == ReqGripperERelease:
            rescode, resdata = self._emergent_release()
        elif reqcode == ReqGripperOpen:
            rescode, resdata = self._open(request.speed, request.force)
        elif reqcode == ReqGripperClose:
            rescode, resdata = self._close(request.speed, request.force)
        elif reqcode == ReqGetGripperState:
            rescode, resdata = self._get_hardware_state()
        elif reqcode == ReqGripperDebug:
            rescode, resdata = self.debug()
        else:
            rescode = self.FAIL
            resdata = 'Not supported request code'
            rospy.logwarn(f'Not supported request code {reqcode}')
        
        response.rescode = rescode
        response.data = resdata
        rospy.loginfo(response)
        return response

    def get_action_state(self):
        rescode = self.FAIL
        resdata = 'Fail'
        if self._robotiq_client.simple_state == actionlib.SimpleGoalState.DONE:
            # 任务完成
            rescode = self.OK
        else:
            # 任务失败
            rescode = self.FAIL
        result: CommandRobotiqGripperResult = self._robotiq_client.get_result()
        resdata = result_to_json_str(result)
        rospy.loginfo(resdata)
        return rescode, resdata

    def _manual_action(self, position, speed, force):
        """
        夹爪正常动作
        """
        rospy.loginfo(f'Manual set gripper at position = {position}, speed = {speed}, force = {force}')
        # 阻塞等待夹爪操作结束
        Robotiq.goto(self._robotiq_client, pos=position, speed=speed, force=force, block=True)
        rospy.sleep(0.1)

        return self.get_action_state()

    def _emergent_release(self):
        """
        夹爪紧急释放
        """
        message = 'Emergency release triggered, after that, gripper server can be served anymore, now respawning gripper server'
        rospy.logwarn(message)
        Robotiq.emergency_release(self._robotiq_client)
        rospy.sleep(0.1)

        # 紧急释放后重启夹爪功能
        rospy.loginfo('Gripper server now respawning ... ')
        servant = rospy.ServiceProxy('node_gripper_daemon_service', SetBool)
        req = SetBoolRequest()
        servant.call(req)
        rospy.signal_shutdown(message)
        
        return self.get_action_state()

    def _stop(self):
        """
        停止夹爪
        """
        rospy.loginfo(f'Stopping the gripper')
        Robotiq.stop(self._robotiq_client, block=True)
        rospy.sleep(0.1)

        return self.get_action_state()

    def _open(self, speed=0.1, force=120):
        """
        打开到最大
        """
        rospy.loginfo(f'Opening the gripper using speed = {speed}, force = {force}')
        Robotiq.open(self._robotiq_client, speed, force, block=True)
        rospy.sleep(1.0)
        
        # 有可能打开失败，打开失败就意味着需要紧急释放了
        # 0.140000000596
        diff = abs(self._gripper_status.position - self._gripper_status.requested_position)
        if self._gripper_status is not None and diff >= 0.0001:
            rospy.logwarn('diff = %.6f, position = %.6f, requested_position = %.6f', diff, self._gripper_status.position, self._gripper_status.requested_position)
            self._emergent_release()

        return self.get_action_state()

    def _close(self, speed=0.1, force=120):
        """
        关闭到最小
        """
        rospy.loginfo(f'Closing the gripper using speed = {speed}, force = {force}')
        Robotiq.close(self._robotiq_client, speed, force)
        rospy.sleep(0.1)

        return self.get_action_state()

    def _get_hardware_state(self):
        """
        获取夹爪硬件现在的状态，包括电流等信息
        """
        # status: RobotiqGripperStatus = self._robotiq_driver.get_current_gripper_status()
        # status.header
        # status.is_ready
        # status.is_reset
        # status.is_moving
        # status.obj_detected
        # status.fault_status
        # status.position
        # status.requested_position
        # status.current
        if self._gripper_status is None:
            return self.FAIL, ''
        else:
            return self.OK, result_to_json_str(self._gripper_status)

    def debug(self):
        rospy.loginfo('DEBUGGING : gripper server running')
        return self.OK, 'Debug information'

    def shutdown_service(self):
        self._robotiq_client.cancel_all_goals()
