#pragma once

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>

// MA2010服务器类
class Ma2010ServerCore {
private:
    moveit::planning_interface::MoveGroupInterfacePtr p_arm_;

public:
    void init();
    bool has_init() const { return p_arm_ != nullptr; }
    moveit::planning_interface::MoveGroupInterfacePtr getArm() const { return p_arm_; }
};