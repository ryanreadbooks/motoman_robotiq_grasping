#include <ros/ros.h>
#include <signal.h>
#include "coordinator/coordinator.h"

Coordinator *p_coordinator = nullptr;

void beforeShutdownHandler(int sig) {
  if (p_coordinator != nullptr) {
    p_coordinator->dump_status_records();
    ROS_INFO("dumped status records");
  }
  ROS_INFO("node_coordinator shutdown.");
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "node_coordinator", ros::init_options::NoSigintHandler);
  signal(SIGINT, beforeShutdownHandler);

  // 获取检测方式
  std::string detection_method = "planar";
  bool success = ros::param::get("~detection_method", detection_method);
  if (!success) {
    detection_method = "planar";
  }
  if (detection_method != "planar" && detection_method != "spatial") {
    ROS_ERROR(
        "Coordinator does support detection method '%s'. Only 'planar' and 'spatial' are "
        "supported. "
        "The default value will be used.",
        detection_method.c_str());
    detection_method = "planar";
  }
  ROS_INFO("Coordinator use detection_method = %s", detection_method.c_str());
  // 获取设置的检测原点位置
  std::string detection_origin = DETECTION_ORIGIN_1;
  ros::param::set("~detection_method", detection_method);
  success = ros::param::get("~detection_origin", detection_origin);
  if (!success) {
    detection_origin = DETECTION_ORIGIN_1;
  }
  if (detection_origin != DETECTION_ORIGIN_1 && detection_origin != DETECTION_ORIGIN_2) {
    ROS_ERROR(
        "Coordinator does support detection origin '%s'. Only '1#' and '2#' are supported. "
        "The default detection origin will be used.",
        detection_origin.c_str());
    detection_origin = DETECTION_ORIGIN_1;
  }
  ROS_INFO("Coordinator use detection_origin = %s", detection_origin.c_str());

  Coordinator coordinator(detection_method == "planar" ? Coordinator::DetectionMethod::Planar
                                                       : Coordinator::DetectionMethod::Spatial,
                          detection_origin);

  ros::AsyncSpinner spinner(8);
  spinner.start();
  p_coordinator = &coordinator;

  ros::waitForShutdown();
}