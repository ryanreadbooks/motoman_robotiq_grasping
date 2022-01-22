#include "coordinator/coordinator.h"
#include "configor/json.hpp" // for json
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <thread>

using namespace configor; // for json

const static int SLEEP_USECS = 500000; // 0.5s

void check_need_throw_run_time_error(bool ret, const std::string &message) {
  ROS_INFO("ret = %s", ret ? "True" : "False");
  if (!ret) {
    throw std::runtime_error(message);
  }
}

Coordinator::Coordinator() : auto_grasp_client_(COORDINATOR_AUTO_GRASP_ACTION_NAME, true) {
  // 初始化模式
  handle_.param<bool>(DEBUG_PARAM_NAME, is_debug_, true);
  // 初始化服务client和subscriber
  ma2010_client_ = handle_.serviceClient<MA2010Service>(MA2010_SERVICE_NAME);
  gripper_client_ = handle_.serviceClient<GripperService>(GRIPPER_SERVICE_NAME);
  detection_client_ = handle_.serviceClient<SetBool>(DETECTION_SERVICE_NAME);
  detection_res_sub_ = handle_.subscribe( DETECTION_TOPIC_RESULT_NAME, 5, &Coordinator::detection_result_cb, this);

  p_tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

  namespace HD = std::placeholders;
  switch_mode_server_ =
      handle_.advertiseService(COORDINATOR_SWITCH_MODE_SERVICE_NAME,
                               &Coordinator::do_switch_mode_service, this);

  debug_run_once_server_ =
      handle_.advertiseService(COORDINATOR_DEBUG_RUN_ONCE_NAME,
                               &Coordinator::do_debug_run_once_service, this);

  start_stop_server_ =
      handle_.advertiseService(COORDINATOR_START_STOP_SERVICE_NAME,
                               &Coordinator::do_start_stop_service, this);

  auto_grasp_server_.reset(new AutoGraspServer(
      handle_, COORDINATOR_AUTO_GRASP_ACTION_NAME,
      boost::bind(&Coordinator::do_auto_grasp_action_request, this, _1),
      false));
  auto_grasp_server_->start();
};

// 会在子线程中回调
void Coordinator::detection_result_cb(const DetectionResultPtr &pd) {
  p_det_res_ = pd;
}

// 只进行一次步进
bool Coordinator::run_once() {
  // 从检测结果中得到抓取目标并且前往
  bool res = false;
  if (p_det_res_ != nullptr && p_det_res_->success) {
    try {
      TransformStamped trans = tf_buffer_.lookupTransform(
          "base_link", "grasp_candidate", ros::Time(0.0));
      double qw = trans.transform.rotation.w;
      double qx = trans.transform.rotation.x;
      double qy = trans.transform.rotation.y;
      double qz = trans.transform.rotation.z;
      double tx = trans.transform.translation.x;
      double ty = trans.transform.translation.y;
      double tz = trans.transform.translation.z;
      ROS_INFO("Grasp pose: [%f, %f, %f, %f, %f, %f, %f] (qw, qx, qy, qz, tx, "
               "ty, tz)",
               qw, qx, qy, qz, tx, ty, tz);
      // 预闭合
      res = set_gripper_width(p_det_res_->grasp_width);
      check_need_throw_run_time_error(res, "Can not set gripper width");
      usleep(SLEEP_USECS);
      // 前往抓取目的地
      res = go_to_target_position(trans);
      check_need_throw_run_time_error(res, "Can not go to target position");
      usleep(SLEEP_USECS);
      // 闭合夹爪
      res = close_gripper();
      check_need_throw_run_time_error(res, "Can not close finger");
      usleep(SLEEP_USECS);
      // 抬起
      res = lift_up_arm();
      check_need_throw_run_time_error(res, "Can not lift up arm");
      sleep(2); // 等待夹爪得到正确的状态
      // 检查是否成功夹取物体
      res = obj_detected_between_fingers();
      check_need_throw_run_time_error(
          res, "No object between fingers, grasp failed!!");
      usleep(SLEEP_USECS);
      // 前往目标地点
      res = go_to_destination();
      check_need_throw_run_time_error(res, "Can not go to destination");
      usleep(SLEEP_USECS);
      // 松开夹爪
      res = open_gripper();
      usleep(SLEEP_USECS);
      // 回到原点
      back_to_origin();
    } catch (std::exception &ex) {
      ROS_ERROR("%s", ex.what());
      p_det_res_ = nullptr;
      open_gripper(); // may throw exception due to gripper server done and cause crash
      back_to_origin();
      return false;
    }
    return true;
  }
}

// 调试模式和自动模式切换
bool Coordinator::do_switch_mode_service(SetBool::Request &req,
                                         SetBool::Response &res) {
  is_debug_ = req.data; // 是否为debug模式
  res.success = true;
  res.message = is_debug_ ? "Successfully set to debug mode"
                          : "Successfully set to auto mode";
  handle_.setParam(DEBUG_PARAM_NAME, is_debug_);
  return true;
}

bool Coordinator::do_debug_run_once_service(Trigger::Request &req,
                                            Trigger::Response &res) {
  // 自动模式下，不能够单独call run_once
  if (!is_debug_) {
    res.success = false;
    res.message = "Can not call run_once service in auto mode";
    return true;
  }
  // 开始run_once，返回是否执行成功
  res.success = run_once();
  res.message = res.success ? "Succeed" : "Failed";
  return true;
}

// 在自动运行情况下，进行开启自动抓取物体的作业
void Coordinator::do_auto_grasp_action_request(const AutoGraspServer::GoalConstPtr &goal) {
  // 不是在自动模式下，不能自己动作
  if (is_debug_) {
    ac_result_.success = false;
    ac_result_.message = "Can not start auto running in debug mode, call service "
                        "coordinator/switch_service and set data to false";
    auto_grasp_server_->setAborted(ac_result_, ac_result_.message);
    return;
  }
  ros::Time st = ros::Time::now();
  is_auto_running_ = true;
  ROS_INFO("Triggered auto pick-and-place, number of objects to grasped is %d, "
           "max attempts is %d", goal->n_object, goal->max_attempts);

  unsigned int n_cur_grasped_objs = 0; // 成功抓取的次数
  unsigned int cur_attempt = 0;        // 当前尝试的次数

  // 当还没有抓取完所有物体，并且还剩余抓取次数才能保持自动运行状态
  while (n_cur_grasped_objs < goal->n_object && cur_attempt < goal->max_attempts) {
    cur_attempt++;
    ROS_INFO("In auto running mode, now in attempt-%d", cur_attempt);
    bool res = run_once();
    if (!res) {
      // 失败
      ROS_WARN("Attempt-%d failed, completion is (%d/%d)", cur_attempt,
               n_cur_grasped_objs, goal->n_object);
    } else {
      // 成功
      n_cur_grasped_objs++;
    }
    sleep(1);
    ROS_INFO("Attempt-%d succeed, current completion is (%d/%d)", cur_attempt,
             n_cur_grasped_objs, goal->n_object);
    // 组织feedback返回结果
    ac_feedback_.n_cur_grasped = n_cur_grasped_objs;
    ac_feedback_.n_attempts_left = goal->max_attempts - cur_attempt;
    auto_grasp_server_->publishFeedback(ac_feedback_);
    if (auto_grasp_server_->isPreemptRequested()) {
      // 停止作业
      ROS_INFO("auto_grasp_server_ PreemptRequested, auto working will stop");
      ac_result_.success = true;
      ac_result_.n_success = n_cur_grasped_objs;
      ac_result_.message = "Operation stopped";
      auto_grasp_server_->setPreempted(ac_result_, ac_result_.message);
      is_auto_running_ = false;
      return;
    }
  }
  if (cur_attempt >= goal->max_attempts) {
    ROS_INFO("Max attempt reached!!!");
  }
  ac_result_.success = true;
  std::stringstream ss;
  double duration = (ros::Time::now() - st).toSec();
  ss << "Successfully grasped " << n_cur_grasped_objs << " objects out of "
     << goal->n_object << " objects in " << duration << " seconds";
  ac_result_.n_success = n_cur_grasped_objs;
  ac_result_.message = ss.str();

  ROS_INFO("Action took %.6f seconds", duration);
  is_auto_running_ = false;
  auto_grasp_server_->setSucceeded(ac_result_, ss.str());
}

bool Coordinator::do_start_stop_service(AutoOperation::Request &req, AutoOperation::Response &res) {
  if (is_debug_) {
    string msg = "Can not start auto running in debug mode, call service "
                 "/coordinator/switch_service and set data to false";
    res.success = false;
    res.message = msg;
    return true;
  }
  if (is_auto_running_ && req.data == "on") {
    res.success = true;
    res.message = "Already auto running";
  } else {
    if (req.data == "on") {
      // 开启
      AutoGraspGoal goal;
      goal.n_object = req.n_object;
      goal.max_attempts = req.max_attempts;
      goal.data = req.data;
      // 给action服务器发送目标
      auto_grasp_client_.sendGoal(goal);  // 非阻塞，直接返回
      res.message = "Start operation success";
    } else if (req.data == "off") {
      // 终止当前作业
      auto_grasp_client_.cancelGoal();
      res.message = "Stop operation requested, wait until current operation finishes";
    }
    res.success = true;
  }
  return true;
}

// 回到检测原点位置
bool Coordinator::back_to_origin() {
  ROS_INFO("Go back to origin");
  Pose tp;
  bool ret = operate_arm(ReqGoDetectionOrigin, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Reached origin");
    return true;
  }
  ROS_WARN("Can not reach origin");
  return false;
}

// 前往抓取目标位置
bool Coordinator::go_to_target_position(
    const geometry_msgs::TransformStamped &t) {
  ROS_INFO("Going to grasp target position");
  Pose target_pose;
  target_pose.position.x = t.transform.translation.x;
  target_pose.position.y = t.transform.translation.y;
  target_pose.position.z = t.transform.translation.z;
  target_pose.orientation.w = t.transform.rotation.w;
  target_pose.orientation.x = t.transform.rotation.x;
  target_pose.orientation.y = t.transform.rotation.y;
  target_pose.orientation.z = t.transform.rotation.z;
  // 对target pose的高度进行一些限制
  target_pose.position.z -= 0.025;
  target_pose.position.z = std::max(0.35, target_pose.position.z);
  bool ret = operate_arm(ReqGoCustomWithPre, target_pose).rescode == ResOK;
  if (ret) {
    ROS_INFO("Successfully moved to target pose");
    return true;
  }
  ROS_WARN("Can not move to target pose");
  return false;
}

// 前往释放点
bool Coordinator::go_to_destination() {
  ROS_INFO("Going to destination");
  Pose tp;
  bool ret = operate_arm(ReqGoDest, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Reached destination");
    return true;
  }
  ROS_WARN("Can not reach destination");
  return false;
}

// 抬起机械臂
bool Coordinator::lift_up_arm() {
  Pose tp;
  bool ret = operate_arm(ReqGoUp, tp).rescode == ResOK;
  if (ret) {
    ROS_INFO("Successfully moved up");
    return true;
  }
  ROS_WARN("Can not move up");
  return false;
}

// 机械臂操作统一请求
MA2010Service::Response Coordinator::operate_arm(int op, Pose &target) {
  MA2010Service ma_servant;
  ma_servant.request.reqcode = op;
  ma_servant.request.target = target;
  bool ret = ma2010_client_.call(ma_servant);
  return ma_servant.response;
}

GripperService::Response Coordinator::operate_gripper(GripperOp op, double width = 0.0) {
  GripperService grip_servant;
  if (op == GripperOp::OPEN) {
    grip_servant.request.reqcode = ReqGripperOpen;
  } else if (op == GripperOp::CLOSE) {
    grip_servant.request.reqcode = ReqGripperClose;
    grip_servant.request.speed = 0.7;
    grip_servant.request.force = 1.3;
  } else if (op == GripperOp::MANUAL) {
    grip_servant.request.position = width;
    grip_servant.request.speed = 1.0;
    grip_servant.request.force = 1.0;
    grip_servant.request.reqcode = ReqGripperManualAction;
  } else {
    grip_servant.request.reqcode = ReqGetGripperState;
  }
  bool ret = gripper_client_.call(grip_servant);

  return grip_servant.response;
}

// 打开夹爪
bool Coordinator::open_gripper() {
  int rescode = operate_gripper(GripperOp::OPEN).rescode;
  ROS_INFO("Open gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::close_gripper() {
  int rescode = operate_gripper(GripperOp::CLOSE).rescode;
  ROS_INFO("Close gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::set_gripper_width(double width) {
  int rescode = operate_gripper(GripperOp::MANUAL, width).rescode;
  ROS_INFO("Manual gripper rescode = %d", rescode);
  return rescode == ResOK;
}

bool Coordinator::obj_detected_between_fingers() {
  string res_data = operate_gripper(GripperOp::GET_STATE).data;
  if (res_data.size() == 0) {
    return false;
  }
  json j = json::parse(res_data);
  return (bool)j["obj_detected"];
}