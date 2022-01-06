#pragma once

#include <iostream>
#include <string>
#include <memory>
#include <vector>
#include <map>
#include <functional>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"
#include "gripper_server/GripperService.h"
#include "gripper_server/gripper_reqres.h"
#include "detection/DetectionResult.h"

using namespace ros;
using geometry_msgs::Pose;
using geometry_msgs::PoseStamped;
using geometry_msgs::TransformStamped;
using std::map;
using std::string;
using std::vector;

using gripper_server::GripperService;
using ma2010_server::MA2010Service;
using std_srvs::SetBool;
using DetectionResultPtr = detection::DetectionResult::ConstPtr;

// 需要用到的服务和话题名
const static string MA2010_SERVICE_NAME = "/node_ma2010_service";
const static string GRIPPER_SERVICE_NAME = "/node_gripper_service";
const static string DETECTION_SERVICE_NAME = "/detection/switch_service";
const static string DETECTION_TOPIC_RESULT_NAME = "/detection/result";


class Coordinator
{
public:
    Coordinator();
    Coordinator(const Coordinator &) = delete;
    Coordinator &operator=(const Coordinator &) = delete;

public:
    void detection_result_cb(const DetectionResultPtr &pd);

    void run();

    void set_running(bool running) { is_running_ = running; }

    bool do_start_running_service(SetBool::Request& req, SetBool::Response& res);

private:
    // 回到检测原点位置
    bool back_to_origin();
    // 前往抓取目标位置
    bool go_to_target_position(const geometry_msgs::TransformStamped&, double);
    // 前往释放点
    bool go_to_destination();
    // 闭合夹爪
    bool close_gripper();
    // 打开夹爪
    bool open_gripper();

private:
    volatile bool is_running_ = true;
    NodeHandle handle_;
    ServiceClient ma2010_client_;
    ServiceClient gripper_client_;
    ServiceClient detection_client_;
    Subscriber detection_res_sub_;
    // ServiceServer coordinator_server_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> p_tf_listener_;
    // cache objects
    DetectionResultPtr pd_;
};