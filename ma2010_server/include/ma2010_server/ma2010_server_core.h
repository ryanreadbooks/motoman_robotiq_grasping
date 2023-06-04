#pragma once

#include <exception>
#include <functional>
#include <map>
#include <ros/ros.h>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/RobotTrajectory.h>

#include "configor/json.hpp" // for json
#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"

using namespace configor; // for json

using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using std::bind;
using std::function;
using std::map;
using std::string;
using std::stringstream;
using std::vector;
using namespace moveit::planning_interface;

using Ma2010Request = ma2010_server::MA2010ServiceRequest;
using Ma2010Response = ma2010_server::MA2010ServiceResponse;

const static string ARM_GROUP = "manipulator";
const static string StrResCode = "rescode";
const static string StrJoints = "joints";
const static string StrComments = "comments";
// 实施检测位置的各个关节角
const static vector<double> DetectionOriginJoints = {
    -0.009186080656945705, -0.2161998003721237, -0.263494074344635,
    0.01527155563235283,   -1.472057342529297,  -0.02905337326228619};
const static vector<double> DestinationJoints = {
    0.6410348415374756, 0.2847989499568939, -0.1091208159923553,
    0.04530277103185654, -1.143249869346619, -0.645852267742157};
// 第二个检测位置
const static vector<double> DetectionOriginJoints_No2 = {
    -0.01233484968543053, -0.6001738309860229, -0.5542929768562317,
    0.01470846962183714, -1.394480347633362, -0.02736466936767101};
const static double MIN_Z = 0.35;

// MA2010服务器类
class Ma2010ServerCore {
  using Ptr = MoveGroupInterfacePtr;
  using PlanPtr = MoveGroupInterface::PlanPtr;

public:
  Ma2010ServerCore();
  Ma2010ServerCore(const Ma2010ServerCore &) = delete;
  Ma2010ServerCore &operator=(const Ma2010ServerCore &) = delete;

public:
  // 初始化
  void init();
  bool has_init() const { return p_arm_ != nullptr; }
  Ptr get_arm() const { return p_arm_; }
  // 处理请求的入口函数
  bool do_request(Ma2010Request &request, Ma2010Response &response);

private:
  void set_target_pose(Pose &pose, const string &message);
  void plan_motion(const string &message);
  void move(const Ma2010Request &, Ma2010Response &, json &);
  void go_home(const Ma2010Request &, Ma2010Response &, json &);
  void go_dest(const Ma2010Request &, Ma2010Response &, json &);
  void go_up(const Ma2010Request &, Ma2010Response &, json &);
  void go_down(const Ma2010Request &, Ma2010Response &, json &);
  void go_detection_origin(const Ma2010Request &, Ma2010Response &, json &);
  void go_detection_origin_no2(const Ma2010Request &, Ma2010Response &, json &);
  void go_custom(const Ma2010Request &, Ma2010Response &, json &);
  void go_custom_with_pre(const Ma2010Request &, Ma2010Response &, json &);
  void get_current_pose(const Ma2010Request &, Ma2010Response &, json &) const;
  void get_current_joints(const Ma2010Request &, Ma2010Response &,
                          json &) const;

private:
  //
  Ptr p_arm_;
  PlanPtr p_plan_;
  // 请求码和请求处理函数的映射
  map<int, function<void(const Ma2010Request &, Ma2010Response &, json &)>>
      request_mapping_;
};