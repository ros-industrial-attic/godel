#include <godel_process_execution/blend_process_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "blend_process_service_node");

  ros::NodeHandle nh;

  godel_process_execution::BlendProcessService process_executor(nh);

  ros::spin();
  return 0;
}
