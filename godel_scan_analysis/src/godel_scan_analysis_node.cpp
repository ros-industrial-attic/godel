#include <ros/ros.h>

#include "godel_scan_analysis/keyence_scan_server.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_server");
  godel_scan_analysis::ScanServer server("world_frame", "keyence_sensor_optical_frame");
  ROS_INFO("ScanServer is online");
  ros::spin();
}