#include "ma2010_server/ma2010_server_core.h"
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
    ROS_INFO("move group for MA2010 manipulator first initialized");
    // 放慢速度
    p_arm_->setMaxAccelerationScalingFactor(0.07);
    p_arm_->setMaxVelocityScalingFactor(0.1);
  }
}

// 请求处理函数
bool Ma2010ServerCore::do_request(Ma2010Request &request,
                                  Ma2010Response &response) {
  int reqcode = request.reqcode;
  ROS_INFO("Got reqcode = %d", reqcode);
  // 记录处理结果
  json res_json; // in configor namespace
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
    response.rescode = ResFail;
    res_json[StrResCode] = ResFail;
    res_json[StrJoints] = {};
    res_json[StrComments] = why;
  }
  response.reqcode = reqcode;
  response.data = res_json.dump(); //序列化为字符串
  ROS_INFO("Response => %s", response.data.c_str());
  return true;
}

// 通过pose的方式设置机械臂运动目标
void Ma2010ServerCore::set_target_pose(Pose &pose, const string &message) {
  // 检查pose的值是否合法
  if (pose.position.x == 0 && pose.position.y == 0 && pose.position.z == 0 &&
      pose.orientation.w == 0 && pose.orientation.x == 0 &&
      pose.orientation.y == 0 && pose.orientation.z == 0) {
    throw std::invalid_argument(
        "Invalid target pose (all zero values received)");
  }
  if (pose.position.z < MIN_Z) {
    ROS_WARN("Requested z value of pose position is %.5f, which may cause "
             "potential collision, "
             "now altering it to minimum value of 0.35", pose.position.z);
    pose.position.z = MIN_Z;
  }
  bool opres = p_arm_->setPoseTarget(pose);
  if (!opres) {
    ROS_ERROR("Can not set pose target for %s!", message.c_str());
    throw std::invalid_argument("Can not set pose target for " + message);
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
  ROS_INFO("%s", "Current joints are :");

  cout << "(-)";
  for (const double& j : p_arm_->getCurrentJointValues()) {
    cout << setprecision(10) << j << " ";
  }
  cout << endl;
  vector<trajectory_msgs::JointTrajectoryPoint> planned_points = p_plan_->trajectory_.joint_trajectory.points;
  ROS_INFO("Successfully planned trajectory with %d points, they are :", planned_points.size());
  for (int i = 0; i < planned_points.size(); ++i)
  {
    cout << "(" << i << ")";
    
    for (const double &value : planned_points[i].positions)
    {
      cout << setprecision(10) << value << " ";
    }
    cout << endl;
  }
}

// 机械臂移动
void Ma2010ServerCore::move(const Ma2010Request &request,
                            Ma2010Response &response, json &res_json) {
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
void Ma2010ServerCore::go_home(const Ma2010Request &request,
                               Ma2010Response &response, json &res_json) {
	ROS_INFO("%s", "Arm going home");
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
void Ma2010ServerCore::go_dest(const Ma2010Request &request,
                               Ma2010Response &response, json &res_json) {
	ROS_INFO("%s", "Arm going destination");
  bool opres = p_arm_->setJointValueTarget(DestinationJoints);
  if (!opres) {
    // 设置失败
    ROS_ERROR("Target joint values are out of bounds when trying to set target "
              "as destination");
    throw std::invalid_argument("Target joint values are out of bounds when "
                                "trying to set target as destination");
  }
  plan_motion("going to destination");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to destination");
}

// 当前位置沿z轴向上
void Ma2010ServerCore::go_up(const Ma2010Request &request,
                             Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going up");
	PoseStamped pose = p_arm_->getCurrentPose();
  pose.pose.position.z += 0.30;
  set_target_pose(pose.pose, "going up");
  plan_motion("going up");
  move(request, response, res_json);
  ROS_INFO("Successfully moved up");
}

// 当前位置沿z轴向下
void Ma2010ServerCore::go_down(const Ma2010Request &request,
                               Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going down");
	PoseStamped pose = p_arm_->getCurrentPose();
  pose.pose.position.z -= 0.20;
  set_target_pose(pose.pose, "going down");
  plan_motion("going down");
  move(request, response, res_json);
  ROS_INFO("Successfully moved down");
}

// 进行检测的位置
void Ma2010ServerCore::go_detection_origin(const Ma2010Request &request,
                                           Ma2010Response &response,
                                           json &res_json) {
  ROS_INFO("%s", "Arm going to detection origin");
	// 用关节角的方式移动到检测原点
  bool opres = p_arm_->setJointValueTarget(DetectionOriginJoints);
  if (!opres) {
    // 设置失败
    ROS_ERROR("Target joint values are out of bounds when trying to set target "
              "as detection origin");
    throw std::invalid_argument("Target joint values are out of bounds when "
                                "trying to set target as detection origin");
  }
  plan_motion("going detection origin");
  // 规划成功
  move(request, response, res_json);
  ROS_INFO("Successfully move to detection origin");
}

// 去到指定的位姿
void Ma2010ServerCore::go_custom(const Ma2010Request &request,
                                 Ma2010Response &response, json &res_json) {
  ROS_INFO("%s", "Arm going to custom position");
	bool opres = p_arm_->setPoseTarget(request.target);
  if (!opres) {
    ROS_ERROR("Can not set pose target!");
    throw std::invalid_argument("Can not set pose target!");
  }
  plan_motion("going to custom pose");
  // move(request, response, res_json);
  ROS_INFO("In custom mode, for debugging, plan only");
  ROS_INFO("Successfully move to target pose");
}

// 前往目标地点，中间经过一个点
void Ma2010ServerCore::go_custom_with_pre(const Ma2010Request &request,
                                          Ma2010Response &response,
                                          json &res_json) {
  ROS_INFO("%s", "Arm going to custom position with pre-point");
	Pose target_pose = request.target;
  Pose pre_target_pose = target_pose;
  pre_target_pose.position.z += 0.30;
  set_target_pose(pre_target_pose, "going to pre-target point");
  plan_motion("going to pre-target point");
  move(request, response, res_json);
  usleep(100000);
  set_target_pose(target_pose, "going to target point");
  plan_motion("going to target point");
  move(request, response, res_json);
  // vector<Pose> waypoints = {pre_target_pose, target_pose};
  // moveit_msgs::RobotTrajectory trajectory;
  // const double eef_step = 0.01;
  // const double jump_threshold = 0.0;
  // int attempts = 0, max_tries = 10;
  // double fraction = 0.0;
  // while (fraction < 1.0 && attempts < max_tries) {
  //   fraction = p_arm_->computeCartesianPath(waypoints, eef_step,
  //   jump_threshold,
  //                                           trajectory);
  //   attempts++;
  // }
  // if (fraction == 1.0) {
  //   ROS_INFO("Successfully planned cartesian trajectory.");
  //   // p_plan_->trajectory_ = trajectory;
  //   moveit::planning_interface::MoveGroupInterface::Plan plan;
  //   plan.trajectory_ = trajectory;
  //   if (p_arm_->execute(plan) ==
  //   moveit::planning_interface::MoveItErrorCode::SUCCESS) {
  //   	get_current_joints(request, response, res_json);
  // 	} else {
  // 		throw std::runtime_error("Can not follow the trajectory, motion
  // failed!");
  // 	}
  // } else {
  //   throw std::logic_error("Can not plan trajectory");
  // }
}

// 获取当前末端位姿
void Ma2010ServerCore::get_current_pose(const Ma2010Request &request,
                                        Ma2010Response &response,
                                        json &res_json) const {
  get_current_joints(request, response, res_json);
}

// 获取当前的关节角
void Ma2010ServerCore::get_current_joints(const Ma2010Request &request,
                                          Ma2010Response &response,
                                          json &res_json) const {
  // 获取pose
  response.curstate = p_arm_->getCurrentPose();
  // 六个关节角
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