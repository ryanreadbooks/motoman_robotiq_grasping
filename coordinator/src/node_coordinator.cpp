#include <ros/ros.h>
#include "coordinator/coordinator.h"


int main(int argc, char** argv){
    ros::init( argc, argv, "node_coordinator");

    Coordinator coordinator;

    ros::AsyncSpinner spinner(3);
    spinner.start();
    coordinator.run();

    ros::waitForShutdown();
    // ros::spin();
}