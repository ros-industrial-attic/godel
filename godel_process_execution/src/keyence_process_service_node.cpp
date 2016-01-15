#include <godel_process_execution/keyence_process_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyence_process_service_node");
  ros::NodeHandle nh;

  godel_process_execution::KeyenceProcessService process_executor(nh);

  ros::spin();
  return 0;
}
