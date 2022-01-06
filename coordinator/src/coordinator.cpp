#include <thread>
#include <tf2_ros/buffer_interface.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "coordinator/coordinator.h"
#include "configor/json.hpp"    // for json

using namespace configor;   // for json


Coordinator::Coordinator()
{
    // 初始化服务client和subsriber
    ma2010_client_ = handle_.serviceClient<MA2010Service>(MA2010_SERVICE_NAME);
    gripper_client_ = handle_.serviceClient<GripperService>(GRIPPER_SERVICE_NAME);
    detection_client_ = handle_.serviceClient<SetBool>(DETECTION_SERVICE_NAME);
    detection_res_sub_ = handle_.subscribe(DETECTION_TOPIC_RESULT_NAME, 5, &Coordinator::detection_result_cb, this);
    p_tf_listener_.reset(new tf2_ros::TransformListener(tf_buffer_));

};

// 会在子线程中回调
void Coordinator::detection_result_cb(const DetectionResultPtr &pd) {
    // debug 测试
    pd_ = pd;
    ROS_INFO("success = %d, grasp width=%f, message=%s", pd->success, pd->grasp_width, pd->message.c_str());
    try {
        if (tf_buffer_.canTransform("base_link", "grasp_candidate", ros::Time(0))) {
            TransformStamped trans = tf_buffer_.lookupTransform("base_link", "grasp_candidate", ros::Time(0));
            double qw = trans.transform.rotation.w;
            double qx = trans.transform.rotation.x;
            double qy = trans.transform.rotation.y;
            double qz = trans.transform.rotation.z;
            double tx = trans.transform.translation.x;
            double ty = trans.transform.translation.y;
            double tz = trans.transform.translation.z;
            ROS_INFO("base_link->grasp_candidate: (%.5f, %.5f, %.5f, %.5f, %.5f, %.5f, %.5f)", qw, qx, qy, qz, tx, ty, tz);
        }
        else
        {
            ROS_WARN("Can not find transform from base_link to grasp_candidate!");
        }
    } catch (tf2::LookupException& ex) {
        ROS_WARN("%s", ex.what());
    }
}


void Coordinator::run() {
    while (is_running_) {
        sleep(1);
        // 从检测结果中得到抓取目标并且前往
        bool res = false;
        if (pd_ != nullptr && pd_->success)
        {
            TransformStamped trans;
            try
            {
                trans = tf_buffer_.lookupTransform("base_link", "grasp_candidate", ros::Time(0.0));
                double w = trans.transform.rotation.w;
                double x = trans.transform.rotation.x;
                double y = trans.transform.rotation.y;
                double z = trans.transform.rotation.z;
                double tx = trans.transform.translation.x;
                double ty = trans.transform.translation.y;
                double tz = trans.transform.translation.z;
                // TODO 这里为了debug，先continue，不要进行后面的操作
                continue;
                ROS_INFO("Grasp pose: [%f, %f, %f, %f, %f, %f, %f] (qw, qx, qy, qz, tx, ty, tz)", w, x, y, z, tx, ty, tz);
            }
            catch (std::exception &ex)
            {
                ROS_ERROR("%s", ex.what());
            }

            res = go_to_target_position(trans, pd_->grasp_width);
            // 前往目的地
            if (!res) {
                ROS_ERROR("Can not move to target position, now exiting...");
                break;
            }
            res = close_gripper();
            if (!res) {
                // 抓取失败，则回到原点重新开始流程
                ROS_WARN("Can not grasp the object, now returing to the origin");
                break;
            }
            // 抓取成功，移动到释放点
            res = go_to_destination();
            if (!res) {
                ROS_WARN("Can not go to destination");
                break;
            }
            // 成功移动到了释放点，开始释放夹爪
            open_gripper();
            // 回到原点
            back_to_origin();
        }
        pd_ = nullptr;
    }
    back_to_origin();
}


// 回到检测原点位置
bool Coordinator::back_to_origin() {
    ROS_INFO("Go back to origin");
    MA2010Service ma_servant;
    ma_servant.request.reqcode = ReqGoDetectionOrigin;
    bool ret = ma2010_client_.call(ma_servant);
    if (ret && ma_servant.response.rescode == ResOK) {
        ROS_INFO("Reached detection origin point.");
        return true;
    }
    return false;
}

// 前往抓取目标位置
bool Coordinator::go_to_target_position(const geometry_msgs::TransformStamped& t, double width) {
    ROS_INFO("Going to grasp target position");
    // 夹爪先闭合到一定的宽度
    GripperService grip_servant;
    grip_servant.request.reqcode = 200;
    grip_servant.request.position = width;
    grip_servant.request.speed = 1.0;
    grip_servant.request.force = 0.5;
    bool ret = gripper_client_.call(grip_servant);
    if (grip_servant.response.rescode != 200) {
        ROS_WARN("Gripper pre-close failed");
    }
    MA2010Service servant;
    servant.request.reqcode = ReqGoCustom;
    Pose target_pose;
    target_pose.position.x = t.transform.translation.x;
    target_pose.position.y = t.transform.translation.y;
    target_pose.position.z = t.transform.translation.z;
    target_pose.orientation.w = t.transform.rotation.w;
    target_pose.orientation.x = t.transform.rotation.x;
    target_pose.orientation.y = t.transform.rotation.y;
    target_pose.orientation.z = t.transform.rotation.z;
    // 对target pose的高度进行一些限制
    servant.request.target = target_pose;
    ret = ma2010_client_.call(servant);
    if (ret && servant.response.rescode == ResOK) {
        // 成功移动到目标位置
        return true;
    }

    return false;
}

// 前往释放点
bool Coordinator::go_to_destination() {
    ROS_INFO("Going to destination");
    MA2010Service ma_servant;
    ma_servant.request.reqcode = ReqGoDest;
    bool ret = ma2010_client_.call(ma_servant);
    if (ret && ma_servant.response.rescode == ResOK) {
        ROS_INFO("Reached destination.");
        return true;
    }
    return false;
}

// 闭合夹爪
bool Coordinator::close_gripper() {
    ROS_INFO("Closing gripper");
    GripperService grip_servant;
    grip_servant.request.reqcode = 202;
    grip_servant.request.speed = 1.0;
    grip_servant.request.force = 5.0;
    bool ret = gripper_client_.call(grip_servant);
    if (ret && grip_servant.response.rescode == 200) {
        // 抬起末端，检查是否确实成功夹持物体
        MA2010Service ma_servant;
        ma_servant.request.reqcode = ReqGetCurPose;
        ret = ma2010_client_.call(ma_servant);
        if (ret && ma_servant.response.rescode == ResOK) {
            PoseStamped cur_pose = ma_servant.response.curstate;
            Pose up_pose = cur_pose.pose;
            up_pose.position.z += 0.3;  // 提升30cm
            ma_servant.request.reqcode = ReqGoCustom;
            ma_servant.request.target = up_pose;
            ret = ma2010_client_.call(ma_servant);
            if (ret && ma_servant.response.rescode == ResOK) {
                ROS_INFO("Lift up arm ..");
                // 检查夹爪是否抓到了物体
                grip_servant.request.reqcode = 205;
                ret = gripper_client_.call(grip_servant);
                if (ret && grip_servant.response.rescode == 200) {
                    // json字符串中提取出obj_detected字段的值
                    string res_json_str = grip_servant.response.data;
                    json j = json::parse(res_json_str);
                    bool obj_detected = (bool)j["obj_detected"];
                    if (obj_detected) {
                        ROS_INFO("Successfully grasp the object.");
                        return true;
                    }
                }
            }
        }
    }
    return false;
}

// 打开夹爪
bool Coordinator::open_gripper() {
    ROS_INFO("Opening gripper");
    GripperService grip_servant;
    grip_servant.request.reqcode = 201;
    bool ret = gripper_client_.call(grip_servant);
    return ret;
}