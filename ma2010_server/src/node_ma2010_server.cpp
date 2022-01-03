#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>
#include <iostream>

#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"
#include "ma2010_server/ma2010_server_core.h"

using std::stringstream;
using namespace ros;
using geometry_msgs::PoseStamped;
using std::cout;
using std::endl;
using std::string;

using Ma2010Request = ma2010_server::MA2010ServiceRequest;
using Ma2010Response = ma2010_server::MA2010ServiceResponse;

const static string NODE_MA2010_NAME = "node_ma2010_server";
const static string ARM_GROUP = "manipulator";
const static string MA2010_SERVICE_NAME = "node_ma2010_service";

Ma2010ServerCore arm_server;

bool do_ma2010_service_request(Ma2010Request& request, Ma2010Response& response) {
    int reqcode = request.reqcode;
    arm_server.init();
    auto arm = arm_server.getArm();
    if (reqcode == ReqGetCurPose)
    {
        PoseStamped cur_pose = arm->getCurrentPose();
        response.curstate = cur_pose;
        response.rescode = ResOK;
    }
    for (const auto& name : arm->getJointNames()) {
        cout << name << " ";
    }
    cout << endl;
    for (const double& v : arm->getCurrentJointValues()) {
        cout << v << " ";
    }
    cout << endl;
    // 组织响应结果
    response.reqcode = reqcode;

    return true;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, NODE_MA2010_NAME);

    NodeHandle handle;
    ServiceServer server = handle.advertiseService(MA2010_SERVICE_NAME, do_ma2010_service_request);
    ROS_INFO("MA2010 Server is on now...");

    // moveit需要用到在用到MoveGroupInterface中的一些函数时，需要ros::AsyncSpinner，否则会失败
    ros::AsyncSpinner spinner(1);
    spinner.start();

    while (ros::ok()) {
        ros::spinOnce();
    }

    return 0;
}