#include <godel_process_execution/abb_blend_process_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_blend_process_service_node");

  ros::NodeHandle nh;

  godel_process_execution::AbbBlendProcessService process_executor(nh);

  ros::spin();
  return 0;
}
