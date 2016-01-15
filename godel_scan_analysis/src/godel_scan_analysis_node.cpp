#include <ros/ros.h>

#include "godel_scan_analysis/keyence_scan_server.h"

const static std::string DEFAULT_WORLD_FRAME = "world_frame";
const static std::string DEFAULT_SCAN_FRAME = "keyence_sensor_optical_frame";
const static double VOXEL_GRID_LEAF_SIZE = 0.005;    // 5 mm
const static double VOXEL_GRID_PUBLISH_PERIOD = 2.0; // seconds

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_server");
  ros::NodeHandle pnh("~");

  // Populate a ScanServerConfiguration
  godel_scan_analysis::ScanServerConfig config;

  // Required params
  if (!pnh.getParam("world_frame", config.world_frame))
  {
    ROS_ERROR("Godel Scan Server requires the 'world_frame' parameter to be set.");
    return -1;
  }

  if (!pnh.getParam("scan_frame", config.scan_frame))
  {
    ROS_ERROR("Godel Scan Server requires the 'scan_frame' parameter to be set.");
    return -1;
  }

  // Optional params
  pnh.param<double>("voxel_leaf_size", config.voxel_grid_leaf_size, VOXEL_GRID_LEAF_SIZE);
  pnh.param<double>("voxel_publish_period", config.voxel_grid_publish_period,
                    VOXEL_GRID_PUBLISH_PERIOD);

  godel_scan_analysis::ScanServer server(config);

  ROS_INFO("Godel scan server is online");

  ros::spin();
}
