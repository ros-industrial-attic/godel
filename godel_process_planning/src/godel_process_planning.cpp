#include "godel_process_planning/godel_process_planning.h"
#include <moveit/robot_model_loader/robot_model_loader.h>

godel_process_planning::ProcessPlanningManager::ProcessPlanningManager(
    const std::string& world_frame, const std::string& blend_group, const std::string& blend_tcp,
    const std::string& keyence_group, const std::string& keyence_tcp,
    const std::string& robot_model_plugin)
    : plugin_loader_("descartes_core", "descartes_core::RobotModel"),
      blend_group_name_(blend_group), keyence_group_name_(keyence_group)
{
  // Attempt to load and initialize the blending robot model
  blend_model_ = plugin_loader_.createInstance(robot_model_plugin);
  if (!blend_model_)
  {
    throw std::runtime_error(std::string("Could not load: ") + robot_model_plugin);
  }

  if (!blend_model_->initialize("robot_description", blend_group, world_frame, blend_tcp))
  {
    throw std::runtime_error("Unable to initialize blending robot model");
  }

  // Attempt to load and initialize the scanning/keyence robot model
  keyence_model_ = plugin_loader_.createInstance(robot_model_plugin);
  if (!keyence_model_)
  {
    throw std::runtime_error(std::string("Could not load: ") + robot_model_plugin);
  }

  if (!keyence_model_->initialize("robot_description", keyence_group, world_frame, keyence_tcp))
  {
    throw std::runtime_error("Unable to initialize scanning robot model");
  }

  // Load the moveit model
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  moveit_model_ = robot_model_loader.getModel();

  if (moveit_model_.get() == NULL)
  {
    throw std::runtime_error("Could not load moveit robot model");
  }
}
