#include <sstream>
#include <exception>
#include "ma2010_server/ma2010_server_core.h"

using std::stringstream;

void Ma2010ServerCore::init() {
    if (!p_arm_) {
        auto arm = new MoveGroupInterface(ARM_GROUP);
        p_arm_.reset(arm);
        auto plan = new MoveGroupInterface::Plan();
        p_plan_.reset(plan);
        ROS_INFO("move group for MA2010 manipulator first initialized");
        // 放慢速度
        p_arm_->setMaxAccelerationScalingFactor(0.07);
        p_arm_->setMaxVelocityScalingFactor(0.15);
    }
}


// 请求处理函数
bool Ma2010ServerCore::do_request(Ma2010Request& request, Ma2010Response& response) {
    int reqcode = request.reqcode;
    ROS_INFO("Got reqcode = %d", reqcode);
    // 记录处理结果
    json res_json;  // in configor namespace
    try
    {
        if (!has_init()) {
            init();
        }
        // 按照请求码处理请求
        if (reqcode == ReqGoHome) {
            go_home(response, res_json);
        } else if (reqcode == ReqGoDest) {
            go_dest(response, res_json);
        } else if (reqcode == ReqGoDetectionOrigin) {
            go_detection_origin(response, res_json);
        } else if (reqcode == ReqGoCustom) {
            go_custom(request.target, response, res_json);
        } else if (reqcode == ReqGetCurPose) {
            get_current_pose(response, res_json);
        } else if (reqcode == ReqGetCurJoints) {
            get_current_joints(response, res_json);
        } else {
            // 不支持的请求码
            throw std::invalid_argument("Not supported request code!");
        }
        response.rescode = ResOK;
    }
    catch (std::exception &ex)  
    {
        // 抛异常表示失败
        auto why = ex.what();
        response.rescode = ResFail;
        res_json[StrResCode] = ResFail;
        res_json[StrJoints] = {};
        res_json[StrComments] = why;
    }
    response.reqcode = reqcode;
    response.data = res_json.dump();//序列化为字符串
    ROS_INFO("Response => %s", response.data.c_str());
    return true;
}


void Ma2010ServerCore::move(Ma2010Response& response, json& res_json) {
    bool success = (p_arm_->move()) == MoveItErrorCode::SUCCESS;
    ROS_INFO("p_arm_->move() result = %s",  success ? "Success" : "Failed");
    if (!success) {
        // 失败
        ROS_ERROR("Can not move to target position");
        throw std::runtime_error("Can not move to target position");
    }
    else
    {
        // 成功
        get_current_joints(response, res_json);
    }
}

// 回到机械臂原点
void Ma2010ServerCore::go_home(Ma2010Response& response, json& res_json) {
    bool res = p_arm_->setNamedTarget("arm_home");
    if (!res) {
        ROS_ERROR("Can not set named target!");
        throw std::invalid_argument("Can not set named target!");
    }
    bool success = (p_arm_->plan(*p_plan_)) == MoveItErrorCode::SUCCESS;
    if (!success) {
        // 规划失败
        ROS_ERROR("Plan failed! Can not plan motion to arm home");
        throw std::logic_error("Plan failed! Can not plan motion to arm home");
    }
    // 规划成功
    move(response, res_json);
    ROS_INFO("Successfully move to arm home");
}

// 去到预设定的目标点
void Ma2010ServerCore::go_dest(Ma2010Response& response, json& res_json) {
    // TODO 找一个目标姿态后完成这个地方
}

// 进行检测的位置
void Ma2010ServerCore::go_detection_origin(Ma2010Response& response, json& res_json) {
    // 用关节角的方式移动到检测原点
    bool opres = p_arm_->setJointValueTarget(DetectionOriginJoints);
    if (!opres) {
        // 设置失败
        ROS_ERROR("Target joint values are out of bounds when trying to set target as detection origin");
        throw std::invalid_argument("Target joint values are out of bounds when trying to set target as detection origin");
    }
    // 关节角设置成功，进行规划
    bool success = (p_arm_->plan(*p_plan_)) == MoveItErrorCode::SUCCESS;
    if (!success) {
        // 规划失败
        ROS_ERROR("Plan failed! Can not plan motion given target joint values");
        throw std::logic_error("Plan failed! Can not plan motion given target joint values");
    }
    // 规划成功
    move(response, res_json);
    ROS_INFO("Successfully move to detection origin");
}

// 去到指定的位姿
void Ma2010ServerCore::go_custom(const Pose& target, Ma2010Response& response, json& res_json) {
    bool opres = p_arm_->setPoseTarget(target);
    if (!opres) {
        ROS_ERROR("Can not set pose target!");
        throw std::invalid_argument("Can not set pose target!");
    }
    bool success = (p_arm_->plan(*p_plan_)) == MoveItErrorCode::SUCCESS;
    if (!success) {
        // 规划失败
        ROS_ERROR("Plan failed! Can not plan motion given target pose");
        throw std::logic_error("Plan failed! Can not plan motion given target pose");
    }
    // move(response, res_json);
    ROS_INFO("In custom mode, for debugging, plan only");
    ROS_INFO("Successfully move to target pose");
}

// 获取当前末端位姿
void Ma2010ServerCore::get_current_pose(Ma2010Response& response, json& res_json) const {
    get_current_joints(response, res_json);
}


// 获取当前的关节角
void Ma2010ServerCore::get_current_joints(Ma2010Response& response, json& res_json) const {
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