#include <ros/ros.h>

#include "godel_scan_analysis/keyence_scan_server.h"

const static std::string DEFAULT_WORLD_FRAME = "world_frame";
const static std::string DEFAULT_SCAN_FRAME = "keyence_sensor_optical_frame";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_server");
  ros::NodeHandle pnh ("~");

  // Load local parameters, if available
  std::string world_frame;
  std::string scan_frame;

  pnh.param<std::string>("world_frame", world_frame, DEFAULT_WORLD_FRAME);
  pnh.param<std::string>("scan_frame", scan_frame, DEFAULT_SCAN_FRAME);

  godel_scan_analysis::ScanServer server(world_frame, scan_frame);

  ROS_INFO("Godel scan server is online");

  ros::spin();
}
