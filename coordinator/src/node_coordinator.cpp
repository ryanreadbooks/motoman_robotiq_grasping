#include "coordinator/coordinator.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_coordinator");
  
  Coordinator coordinator;

  ros::AsyncSpinner spinner(6);
  spinner.start();

  ros::waitForShutdown();
}