#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <moveit/move_group_interface/move_group_interface.h>
#include <ros/ros.h>
#include <sstream>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <string>

#include "ma2010_server/MA2010Service.h"
#include "ma2010_server/ma2010_reqres.h"
#include "ma2010_server/ma2010_server_core.h"

using std::stringstream;
using namespace ros;
using std::cout;
using std::endl;
using std::string;

const static string NODE_MA2010_NAME = "node_ma2010_server";
const static string MA2010_SERVICE_NAME = "node_ma2010_service";

Ma2010ServerCore arm_server;

bool do_ma2010_service_request(Ma2010Request &request,
                               Ma2010Response &response) {
  return arm_server.do_request(request, response);
};

int main(int argc, char **argv) {
  ros::init(argc, argv, NODE_MA2010_NAME);

  NodeHandle handle;
  ServiceServer server =
      handle.advertiseService(MA2010_SERVICE_NAME, do_ma2010_service_request);

  // moveit需要用到在用到MoveGroupInterface中的一些函数时，需要ros::AsyncSpinner，否则会失败
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // 开机械臂使能
  ros::ServiceClient client =
      handle.serviceClient<std_srvs::Trigger>("robot_enable");
  std_srvs::Trigger trigger;
  bool flag = client.call(trigger);
  if (flag) {
    ROS_INFO("Robot enable => Success: %s, Message: %s",
             trigger.response.success == 1 ? "True" : "False",
             trigger.response.message.c_str());
    ROS_INFO("MA2010 Server is on now...");
  } else {
    ROS_WARN("Robot can not be enabled. Try again !");
  }

  while (ros::ok()) {
    ros::spinOnce();
  }

  return 0;
}