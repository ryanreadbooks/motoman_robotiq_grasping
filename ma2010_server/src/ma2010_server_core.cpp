#include "ma2010_server/ma2010_server_core.h"
#include <exception>
#include <iostream>

using std::cout;
using std::endl;
using std::setprecision;

Ma2010ServerCore::Ma2010ServerCore() {
  // 初始化request_mapping_
  namespace HD = std::placeholders;
  request_mapping_ = {
      {ReqGoHome, bind(&Ma2010ServerCore::go_home, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoDest, bind(&Ma2010ServerCore::go_dest, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoDetectionOrigin, bind(&Ma2010ServerCore::go_detection_origin, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoDetectionOrigin2, bind(&Ma2010ServerCore::go_detection_origin_no2, this, HD::_1, HD::_2, HD::_3)},
      {ReqGetCurPose, bind(&Ma2010ServerCore::get_current_pose, this, HD::_1, HD::_2, HD::_3)},
      {ReqGetCurJoints, bind(&Ma2010ServerCore::get_current_joints, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoUp, bind(&Ma2010ServerCore::go_up, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoDown, bind(&Ma2010ServerCore::go_down, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoCustom, bind(&Ma2010ServerCore::go_custom, this, HD::_1, HD::_2, HD::_3)},
      {ReqGoCustomWithPre, bind(&Ma2010ServerCore::go_custom_with_pre, this, HD::_1, HD::_2, HD::_3)},
  };
}

// 初始化
void Ma2010ServerCore::init() {
  if (!p_arm_) {
    auto arm = new MoveGroupInterface(ARM_GROUP);
    p_arm_.reset(arm);
    auto plan = new MoveGroupInterface::Plan();
    p_plan_.reset(plan);
    ROS_INFO("Move group for MA2010 manipulator first initialized.");
    // 放慢速度
    p_arm_->setMaxAccelerationScalingFactor(0.01);
    p_arm_->setMaxVelocityScalingFactor(0.05);
    p_arm_->allowReplanning(true);
  }
}

// 请求处理函数
bool Ma2010ServerCore::do_request(Ma2010Request &request, Ma2010Response &response) {
  int reqcode = request.reqcode;
  ROS_INFO("Got reqcode = %d", reqcode);
  // 记录处理结果
  json res_json;  // in configor namespace
  try {
    if (!has_init()) {
      init();
    }
    // 按照请求码处理请求
    if (!request_mapping_.count(reqcode)) {
      // 不支持的请求码
      throw std::invalid_argument("Not supported request code!");
    }
    request_mapping_[reqcode](request, response, res_json);
    response.rescode = ResOK;
  } catch (std::exception &ex) {
    // 抛异常表示失败
    auto why = ex.what();
    ROS_ERROR("Exception occur during handling request. %s", why);
    response.rescode = ResFail;
    res_json[StrResCode] = ResFail;
    res_json[StrJoints] = {};
    res_json[StrComments] = why;
  }
  response.reqcode = reqcode;
  response.data = res_json.dump();  //序列化为字符串
  ROS_INFO("Response => %s", response.data.c_str());
  return true;
}

// 通过pose的方式设置机械臂运动目标
void Ma2010ServerCore::set_target_pose(Pose &pose, const string &message) {
  p_arm_->clearPoseTarget();
  // 检查pose的值是否合法
  if (pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 && pose.orientation.w == 0 &&
      pose.orientation.x == 0 && pose.orientation.y == 0 && pose.orientation.z == 0) {
    throw std::invalid_argument("Invalid target pose (all zero values received)");
  }
  std::stringstream ss;
  ss << "Ma2010ServerCore::set_target_pose, requested target pose (qw,qx,qy,qz,x,y,z) = "
     << pose.orientation.w << " " << pose.orientation.x << " " << pose.orientation.y << " "
     << pose.orientation.z << " " << pose.position.x << " " << pose.position.y << " " << pose.position.z
     << " ";
  ROS_INFO("%s", ss.str().c_str());

  if (pose.position.z < MIN_Z) {
    ROS_WARN(
        "Requested z value of pose position is %.5f, which may cause "
        "potential collision, "
        "now altering it to minimum value of %.5f",
        pose.position.z, MIN_Z);
    pose.position.z = MIN_Z;
  }
  // bool opres = p_arm_->setPoseTarget(pose);  // 用这个函数有时容易失败
  bool opres =
      p_arm_->setApproximateJointValueTarget(pose);  // 用求解IK的方法设置关节角，和setPoseTarget的原理不一致
  if (!opres) {
    ROS_ERROR("Can not set pose target for %s!", message.c_str());
    throw std::runtime_error("Can not set pose target for " + message);
  }
}

// 机械臂规划动作
void Ma2010ServerCore::plan_motion(const string &message) {
  bool success = (p_arm_->plan(*p_plan_)) == MoveItErrorCode::SUCCESS;
  if (!success) {
    // 规划失败
    ROS_ERROR("Plan failed! Can not plan motion for %s", message.c_str());
    throw std::logic_error("Plan failed! Can not plan motion for " + message);
  }
  // 显示规划的结果
  std::stringstream ss;
  ss << "(c)";
  for (const double &j : p_arm_->getCurrentJointValues()) {
    ss << setprecision(10) << j << " ";
  }
  ss << endl;
  vector<trajectory_msgs::JointTrajectoryPoint> planned_points = p_plan_->trajectory_.joint_trajectory.points;
  ROS_DEBUG("Successfully planned trajectory with %d points, they are :",
            static_cast<int>(planned_points.size()));
  for (int i = 0; i < planned_points.size(); ++i) {
    ss << "(" << i << ")";

    for (const double &value : planned_points[i].positions) {
      ss << setprecision(10) << value << " ";
    }
    ss << endl;
  }
  ROS_DEBUG("%s", ss.str().c_str());
}

// 机械臂移动
void Ma2010ServerCore::move(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  bool success = (p_arm_->move()) == MoveItErrorCode::SUCCESS;
  ROS_INFO("p_arm_->move() result = %s", success ? "Success" : "Failed");
  if (!success) {
    // 失败
    ROS_ERROR("Can not move to target position");
    throw std::runtime_error("Can not move to target position");
  } else {
    // 成功
    get_current_joints(request, response, res_json);
  }
}

// 回到机械臂原点
void Ma2010ServerCore::go_home(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going home...");
  bool res = p_arm_->setNamedTarget("arm_home");
  if (!res) {
    ROS_ERROR("Can not set named target!");
    throw std::invalid_argument("Can not set named target!");
  }
  plan_motion("going home");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to arm home");
}

// 去到预设定的目标点
void Ma2010ServerCore::go_dest(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going destination...");
  bool opres = p_arm_->setJointValueTarget(DestinationJoints);
  if (!opres) {
    // 设置失败
    ROS_ERROR(
        "Target joint values are out of bounds when trying to set target "
        "as destination");
    throw std::invalid_argument(
        "Target joint values are out of bounds when "
        "trying to set target as destination");
  }
  plan_motion("going to destination");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to destination");
}

// 当前位置沿z轴向上
void Ma2010ServerCore::go_up(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going up...");
  PoseStamped pose = p_arm_->getCurrentPose();
  pose.pose.position.z += 0.30;
  set_target_pose(pose.pose, "going up");
  plan_motion("going up");
  move(request, response, res_json);
  ROS_INFO("Successfully moved up");
}

// 当前位置沿z轴向下
void Ma2010ServerCore::go_down(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going down...");
  PoseStamped pose = p_arm_->getCurrentPose();
  pose.pose.position.z -= 0.20;
  set_target_pose(pose.pose, "going down");
  plan_motion("going down");
  move(request, response, res_json);
  ROS_INFO("Successfully moved down");
}

// 进行检测的位置
void Ma2010ServerCore::go_detection_origin(const Ma2010Request &request, Ma2010Response &response,
                                           json &res_json) {
  ROS_INFO("%s", "Arm going to detection origin");
  // 用关节角的方式移动到检测原点
  bool opres = p_arm_->setJointValueTarget(DetectionOriginJoints);
  if (!opres) {
    // 设置失败
    ROS_ERROR(
        "Target joint values are out of bounds when trying to set target "
        "as detection origin");
    throw std::invalid_argument(
        "Target joint values are out of bounds when "
        "trying to set target as detection origin");
  }
  plan_motion("Going detection origin...");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to detection origin");
}

void Ma2010ServerCore::go_detection_origin_no2(const Ma2010Request &request, Ma2010Response &response,
                                               json &res_json) {
  ROS_INFO("%s", "Arm going to detection origin#2");
  // 用关节角的方式移动到检测原点
  bool opres = p_arm_->setJointValueTarget(DetectionOriginJoints_No2);
  if (!opres) {
    // 设置失败
    ROS_ERROR(
        "Target joint values are out of bounds when trying to set target "
        "as detection origin#2");
    throw std::invalid_argument(
        "Target joint values are out of bounds when "
        "trying to set target as detection origin#2");
  }
  plan_motion("Going detection origin#2");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to detection origin#2");
}

// 去到指定的位姿
void Ma2010ServerCore::go_custom(const Ma2010Request &request, Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going to custom position...");
  // bool opres = p_arm_->setPoseTarget(request.target);
  Pose target_pose = request.target;
  set_target_pose(target_pose, "go custom");
  plan_motion("going to custom pose...");
  move(request, response, res_json);
  ROS_INFO("Successfully move to target pose");
}

// 前往目标地点，中间经过一个点
void Ma2010ServerCore::go_custom_with_pre(const Ma2010Request &request, Ma2010Response &response,
                                          json &res_json) {
  ROS_INFO("%s", "Arm going to custom position with pre-point");
  Pose target_pose = request.target;
  Pose pre_target_pose = target_pose;  // 预抓取点
  if (!request.data.empty()) {         // 可能包含预抓取点的坐标
    ROS_INFO("The request data contains extras data");
    try {
      json data_js = json::parse(request.data);
      if (data_js["pre_grasp_pose"].is_array()) {
        vector<double> pre_grasp_pose = data_js["pre_grasp_pose"];
        // 只改变position不改变orientation
        pre_target_pose.position.x = pre_grasp_pose[0];
        pre_target_pose.position.y = pre_grasp_pose[1];
        pre_target_pose.position.z = pre_grasp_pose[2];
      } else {
        throw std::invalid_argument("pre_grasp_pose not valid, because it's not an array");
      }
    } catch (std::exception &ex) {
      ROS_WARN("Request.data is not empty but can not parse json from it. Error: %s", ex.what());
    }
  } else {
    ROS_INFO("The request has no extra data. MA2010 will perform default pre grasp scheme");
    pre_target_pose.position.z += 0.30;  // 在真正的目标点的上方30cm
  }

  set_target_pose(pre_target_pose, "going to pre-target point");
  plan_motion("going to pre-target point");
  move(request, response, res_json);
  usleep(100 * 1000);
  set_target_pose(target_pose, "going to target point");
  plan_motion("going to target point");
  move(request, response, res_json);
  
  // 用waypoints
  // vector<Pose> waypoints = {pre_target_pose, target_pose};
  // moveit_msgs::RobotTrajectory trajectory;
  // const double jump_threshold = 0.0;
  // const double eef_step = 0.01;
  // double fraction = 0.0;
  // int max_try = 100;
  // int attempts = 0;
  // while (fraction < 1.0 && attempts < max_try) {
  //   fraction = p_arm_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
  //   attempts++;
  // }
  // if (fraction == 1) {
  //   // 生成机械臂的运动规划数据
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   plan.trajectory_ = trajectory;
  //   p_arm_->execute(plan);
  //   sleep(1);
  // }
}

// 获取当前末端位姿
void Ma2010ServerCore::get_current_pose(const Ma2010Request &request, Ma2010Response &response,
                                        json &res_json) const {
  get_current_joints(request, response, res_json);
}

// 获取当前的关节角
void Ma2010ServerCore::get_current_joints(const Ma2010Request &request, Ma2010Response &response,
                                          json &res_json) const {
  // 获取pose
  response.curstate = p_arm_->getCurrentPose();
  vector<double> joints = p_arm_->getCurrentJointValues();
  res_json[StrJoints] = joints;
  if (joints.size() != 0) {
    res_json[StrResCode] = ResOK;
    res_json[StrComments] = "success";
  } else {
    res_json[StrResCode] = ResFail;
    res_json[StrComments] = "failed to retrieve robot joints";
  }
}