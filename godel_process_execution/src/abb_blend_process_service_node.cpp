#include <godel_process_execution/abb_blend_process_service.h>

#include <ros/ros.h>

const static std::string DEFAULT_SERVICE_NAME = "blend_process_execution";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_blend_process_service_node");

  ros::NodeHandle nh;
  ros::NodeHandle pnh ("~");

  std::string sim_service, execution_service;

  if (!pnh.getParam("execution_service", execution_service))
  {
    ROS_ERROR("Keyence Process Execution node requires the 'execution_service' parameter be set.");
    return -1;
  }

  if (!pnh.getParam("simulation_service", sim_service))
  {
    ROS_ERROR("Keyence Process Execution node requires the 'simulation_service' parameter be set.");
    return -1;
  }

  std::string service_name; // the name of THIS service
  pnh.param<std::string>("service_name", service_name, DEFAULT_SERVICE_NAME);

  godel_process_execution::AbbBlendProcessExecutionService process_executor(service_name, sim_name, real_name, nh);

  ros::spin();
  return 0;
}
