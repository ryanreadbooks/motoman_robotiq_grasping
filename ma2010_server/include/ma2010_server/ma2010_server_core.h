#pragma once

#include <string>
#include <vector>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"
#include "configor/json.hpp"    // for json

using namespace configor;   // for json

using geometry_msgs::PoseStamped;
using geometry_msgs::Pose;
using std::vector;
using std::string;
using namespace moveit::planning_interface;

using Ma2010Request = ma2010_server::MA2010ServiceRequest;
using Ma2010Response = ma2010_server::MA2010ServiceResponse;

const static string ARM_GROUP = "manipulator";
const static string StrResCode = "rescode";
const static string StrJoints = "joints";
const static string StrComments = "comments";
// 实施检测位置的各个关节角
const static vector<double> DetectionOriginJoints = {-0.009186080656945705, -0.2161998003721237, -0.263494074344635,
                                                    0.01527155563235283, -1.472057342529297, -0.02905337326228619};

// MA2010服务器类
class Ma2010ServerCore {
    using Ptr = MoveGroupInterfacePtr;
    using PlanPtr = MoveGroupInterface::PlanPtr;

public:
    Ma2010ServerCore() {}
    Ma2010ServerCore(const Ma2010ServerCore &) = delete;
    Ma2010ServerCore &operator=(const Ma2010ServerCore &) = delete;

public:
    // 初始化
    void init();
    bool has_init() const { return p_arm_ != nullptr; }
    Ptr get_arm() const { return p_arm_; }
    // 处理请求的入口函数
    bool do_request(Ma2010Request& request, Ma2010Response& response);

private:
    void move(Ma2010Response&, json&);
    void go_home(Ma2010Response&, json&);
    void go_dest(Ma2010Response&, json&);
    void go_detection_origin(Ma2010Response&, json&);
    void go_custom(const Pose &target, Ma2010Response &, json &);
    void get_current_pose(Ma2010Response&, json&) const;
    void get_current_joints(Ma2010Response&, json&) const;

private:
    Ptr p_arm_;
    PlanPtr p_plan_;
};