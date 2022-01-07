#include "coordinator/coordinator.h"
#include "configor/json.hpp" // for json
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer_interface.h>
#include <thread>

using namespace configor; // for json

const static int SLEEP_USECS = 500000;  // 0.5s

void check_need_throw_run_time_error(bool ret, const std::string& message) {
  ROS_INFO("ret = %s", ret ? "True" : "False");
  if (!ret)
  {
    throw std::runtime_error(message);
  }
}

Coordinator::Coordinator() {
  // 初始化服务client和subsriber
  ma2010_client_ = handle_.serviceClient<MA2010Service>(MA2010_SERVICE_NAME);
  gripper_client_ = handle_.serviceClient<GripperService>(GRIPPER_SERVICE_NAME);
  detection_client_ = handle_.serviceClient<SetBool>(DETECTION_SERVICE_NAME);
  detection_res_sub_ = handle_.subscribe(
      DETECTION_TOPIC_RESULT_NAME, 5, &Coordinator::detection_result_cb, this);
  p_tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));
};

// 会在子线程中回调
void Coordinator::detection_result_cb(const DetectionResultPtr &pd) {
  pd_ = pd;
}

// 只进行一次步进
void Coordinator::run_once() {
  // 从检测结果中得到抓取目标并且前往
  bool res = false;
  if (pd_ != nullptr && pd_->success) {
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
      res = set_gripper_width(pd_->grasp_width);
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
      usleep(SLEEP_USECS);
      // 检查是否成功夹取物体
      res = obj_detected_between_fingers();
      check_need_throw_run_time_error(res, "No object between fingers, grasp failed!!");
      usleep(SLEEP_USECS);
      return;
      // 前往目标地点
      res = go_to_destination();
      check_need_throw_run_time_error(res, "Can not go to destination");
      usleep(SLEEP_USECS);
      // 松开夹爪
      res = open_gripper();
      usleep(SLEEP_USECS);
      // 回到原点
      back_to_origin();
    }
    catch (std::exception &ex)
    {
      ROS_ERROR("%s", ex.what());
      pd_ = nullptr;
      open_gripper();
      back_to_origin();
    }
    ROS_INFO("Coordinator::run_once finished");
  }
}

// 回到检测原点位置
bool Coordinator::back_to_origin() {
  ROS_INFO("Go back to origin");
  bool ret = operate_arm(ReqGoDetectionOrigin, Pose()).rescode == ResOK;
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
  bool ret = operate_arm(ReqGoDest, Pose()).rescode == ResOK;
  if (ret) {
    ROS_INFO("Reached destination");
    return true;
  }
  ROS_WARN("Can not reach destination");
  return false;
}

// 抬起机械臂
bool Coordinator::lift_up_arm() {
  bool ret = operate_arm(ReqGoUp, Pose()).rescode == ResOK;
  if (ret) {
    ROS_INFO("Successfully moved up");
    return true;
  }
  ROS_WARN("Can not move up");
  return false;
}

// 机械臂操作统一请求
MA2010Service::Response Coordinator::operate_arm(int op, Pose target) {
  MA2010Service ma_servant;
  ma_servant.request.reqcode = op;
  ma_servant.request.target = target;
  bool ret = ma2010_client_.call(ma_servant);
  return ma_servant.response;
}

GripperService::Response Coordinator::operate_gripper(GripperOp op,
                                                      double width = 0.0) {
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