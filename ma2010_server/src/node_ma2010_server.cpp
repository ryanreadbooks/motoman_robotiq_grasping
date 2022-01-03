#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <string>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <sstream>

#include "ma2010_server/MA2010Service.h"


using std::stringstream;
using namespace ros;

int main(int argc, char** argv) {
    ros::init(argc, argv, "moveit_ma2010_server_test");
    static const std::string ARM_GROUP = "manipulator";

    NodeHandle handle;
    Publisher pub = handle.advertise<std_msgs::String>("ma2010_test", 1000);
    Rate loop_rate(10);

    int count = 0;
    while (ok()) {
        try {
            moveit::planning_interface::MoveGroupInterface arm(ARM_GROUP);
            ROS_INFO_NAMED("tutorial", "End effector link: %s", arm.getEndEffectorLink().c_str());
        } catch (std::exception& ex) {
            ROS_ERROR(ex.what());
        }
        std_msgs::String msg;
        stringstream ss;
        ss << "hello ma2010 " << count;

        msg.data = ss.str();

        pub.publish(msg);
        spinOnce();
        loop_rate.sleep();
        ++count;

    }

    return 0;
}