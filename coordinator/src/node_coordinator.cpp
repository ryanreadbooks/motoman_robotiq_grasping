#include "coordinator/coordinator.h"
#include <ros/ros.h>

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_coordinator");

  Coordinator coordinator;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  sleep(1.5);
  coordinator.run_once();

  ros::waitForShutdown();
  // ros::spin();
}