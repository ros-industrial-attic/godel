#include <godel_process_execution/quelltech_process_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "quelltech_process_service_node");
  ros::NodeHandle nh;

  godel_process_execution::QuelltechProcessService process_executor(nh);

  ros::spin();
  return 0;
}
