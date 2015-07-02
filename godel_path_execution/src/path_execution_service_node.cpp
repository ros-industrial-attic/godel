#include <godel_path_execution/path_execution_service.h>

#include <ros/ros.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "path_execution_service_node");

  ros::NodeHandle nh;

  std::string real_name, sim_name;

  nh.param<std::string>("actual_execution_service", real_name, "execute_kinematic_path");
  nh.param<std::string>("simulated_execution_service", sim_name, "simulation/simulate");

  godel_path_execution::PathExecutionService path_executor("path_execution", sim_name, real_name, nh);

  ros::spin();
  return 0;
}