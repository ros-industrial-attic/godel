#include <ros/ros.h>
// Process Services
#include <godel_process_planning/godel_process_planning.h>

// Globals
const static std::string DEFAULT_BLEND_PLANNING_SERVICE = "blend_process_planning";
const static std::string DEFAULT_KEYENCE_PLANNING_SERVICE = "keyence_process_planning";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "godel_process_planning");

  // Load local parameters
  ros::NodeHandle nh, pnh("~");
  std::string world_frame, blend_group, keyence_group, blend_tcp, keyence_tcp, robot_model_plugin;
  pnh.param<std::string>("world_frame", world_frame, "world_frame");
  pnh.param<std::string>("blend_group", blend_group, "manipulator_tcp");
  pnh.param<std::string>("keyence_group", keyence_group, "manipulator_keyence");
  pnh.param<std::string>("blend_tcp", blend_tcp, "tcp_frame");
  pnh.param<std::string>("keyence_tcp", keyence_tcp, "keyence_tcp_frame");
  pnh.param<std::string>("robot_model_plugin", robot_model_plugin, "");

  // IK Plugin parameter must be specified
  if (robot_model_plugin.empty())
  {
    ROS_ERROR_STREAM("MUST SPECIFY PARAMETER 'robot_model_plugin' for godel_process_planning node");
    return -1;
  }

  using godel_process_planning::ProcessPlanningManager;

  // Creates a planning manager that will create the appropriate planning classes and perform
  // all required initialization. It exposes member functions to handle each kind of processing
  // event.
  ProcessPlanningManager manager(world_frame, blend_group, blend_tcp, keyence_group, keyence_tcp,
                                 robot_model_plugin);
  // Plumb in the appropriate ros services
  ros::ServiceServer blend_server = nh.advertiseService(
      DEFAULT_BLEND_PLANNING_SERVICE, &ProcessPlanningManager::handleBlendPlanning, &manager);
  ros::ServiceServer keyence_server = nh.advertiseService(
      DEFAULT_KEYENCE_PLANNING_SERVICE, &ProcessPlanningManager::handleKeyencePlanning, &manager);

  // Serve and wait for shutdown
  ROS_INFO_STREAM("Godel Process Planning Server Online");
  ros::spin();

  return 0;
}
