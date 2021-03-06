#pragma once

#include <functional>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <sstream>
#include <map>
#include <vector>
#include <memory>

#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "detection/DetectionResult.h"
#include "gripper_server/GripperService.h"
#include "gripper_server/gripper_reqres.h"
#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"
#include "coordinator/AutoGraspAction.h"
#include "coordinator/AutoOperation.h"

using namespace ros;
using namespace coordinator;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using std::map;
using std::shared_ptr;
using std::string;
using std::vector;

using gripper_server::GripperService;
using ma2010_server::MA2010Service;
using std_srvs::SetBool;
using std_srvs::Trigger;
using DetectionResultPtr = detection::DetectionResult::ConstPtr;
using actionlib::SimpleActionClient;
using actionlib::SimpleActionServer;

// 需要用到的服务和话题名
const static string MA2010_SERVICE_NAME = "/node_ma2010_service";
const static string GRIPPER_SERVICE_NAME = "/node_gripper_service";
const static string DETECTION_SERVICE_NAME = "/detection/switch_service";
const static string DETECTION_TOPIC_RESULT_NAME = "/detection/result";
const static string COORDINATOR_SWITCH_MODE_SERVICE_NAME = "/coordinator/switch_service";
const static string COORDINATOR_START_STOP_SERVICE_NAME = "/coordinator/start_stop_auto";
const static string COORDINATOR_AUTO_GRASP_ACTION_NAME = "/coordinator/auto_grasp";
const static string COORDINATOR_DEBUG_RUN_ONCE_NAME = "/coordinator/debug_run_once_service";

const static string DEBUG_PARAM_NAME = "node_coordinator/debug_mode";

class Coordinator {
  using AutoGraspServer = SimpleActionServer<AutoGraspAction>;
  using AutoGraspClient = SimpleActionClient<AutoGraspAction>;

  enum class GripperOp
  {
    OPEN,
    CLOSE,
    GET_STATE,
    MANUAL
  };

public:
  Coordinator();
  Coordinator(const Coordinator &) = delete;
  Coordinator &operator=(const Coordinator &) = delete;

public:
  void detection_result_cb(const DetectionResultPtr &pd);
  // 只运行一次
  bool run_once();
  // 切换模式服务函数
  bool do_switch_mode_service(SetBool::Request &req, SetBool::Response &res);
  // 调试模式下，运行一次
  bool do_debug_run_once_service(Trigger::Request &req, Trigger::Response &res);
  // 自动抓取的action的处理函数
  void do_auto_grasp_action_request(const AutoGraspServer::GoalConstPtr &goal);
  // 处理开始或者停止自动作业的函数
  bool do_start_stop_service(AutoOperation::Request &req, AutoOperation::Response &res);

private:
  // 操作机械臂
  MA2010Service::Response operate_arm(int, Pose&);
  // 回到检测原点位置
  bool back_to_origin();
  // 前往抓取目标位置
  bool go_to_target_position(const geometry_msgs::TransformStamped &);
  // 前往释放点
  bool go_to_destination();
  // 抬起机械臂
  bool lift_up_arm();

  // 操作夹爪
  GripperService::Response operate_gripper(GripperOp, double);
  // 打开夹爪
  bool open_gripper();
  // 关闭夹爪
  bool close_gripper();
  // 将夹爪宽度设置到设定宽度
  bool set_gripper_width(double);
  // 检查两指间是否有物体
  bool obj_detected_between_fingers();

private:
  // 标记是否为调试模式
  bool is_debug_ = true;
  // 在自动模式下，标记是否正在运行
  bool is_auto_running_ = false;
  NodeHandle handle_;
  ServiceClient ma2010_client_;
  ServiceClient gripper_client_;
  ServiceClient detection_client_;
  Subscriber detection_res_sub_;

  // 切换模式服务
  ServiceServer switch_mode_server_;
  // 调试模式下，触发运动一次的服务
  ServiceServer debug_run_once_server_;
  // 开启或者停止自动作业
  ServiceServer start_stop_server_;
  // 自动抓取的action服务器
  shared_ptr<AutoGraspServer> auto_grasp_server_;
  AutoGraspResult ac_result_;
  AutoGraspFeedback ac_feedback_;
  // action客户端
  AutoGraspClient auto_grasp_client_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> p_tf_listener_;
  // cache objects
  DetectionResultPtr p_det_res_;
};