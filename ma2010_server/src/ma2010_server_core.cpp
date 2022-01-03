#include "ma2010_server/ma2010_server_core.h"


void Ma2010ServerCore::init() {
    if (!p_arm_) {
        auto arm = new moveit::planning_interface::MoveGroupInterface("manipulator");
        p_arm_.reset(arm);
        ROS_INFO("move group for MA2010 manipulator first initialized");
    }
}