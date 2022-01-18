#!/bin/bash

cd ../..
source devel/setup.bash

roslaunch gripper_server bringup_gripper_server.launch &
roslaunch gripper_server bringup_gripper_server_daemon.launch