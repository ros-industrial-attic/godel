#ifndef GODEL_PROCESS_PLANNING_H
#define GODEL_PROCESS_PLANNING_H

#include "godel_msgs/BlendProcessPlanning.h"
#include "godel_msgs/KeyenceProcessPlanning.h"

#include <descartes_core/robot_model.h>
#include <pluginlib/class_loader.h>

/*
 * This class wraps Descartes planning methods and provides functionality for configuration
 * and for planning for blending/scanning paths.
 *
 * The general plannning approach is:
 *
 * 1. Find the closest starting point for robot between where it is now and the first point
 *    in the trajectory.
 * 2. Create a linear-joint motion to the start point, append the path, and add a linear
 *    path home
 * 3. Make a rough plan that moves through these points and solve
 * 4. Replan for the approach and departure using this plan as a 'seed'. Only at this point
 *    are collisions considered.
 */
namespace godel_process_planning
{

class ProcessPlanningManager
{
public:
  ProcessPlanningManager(const std::string& world_frame, const std::string& blend_group,
                         const std::string& blend_tcp, const std::string& keyence_group,
                         const std::string& keyence_tcp, const std::string& robot_model_plugin);

  bool handleBlendPlanning(godel_msgs::BlendProcessPlanning::Request& req,
                           godel_msgs::BlendProcessPlanning::Response& res);

  bool handleKeyencePlanning(godel_msgs::KeyenceProcessPlanning::Request& req,
                             godel_msgs::KeyenceProcessPlanning::Response& res);

private:
  descartes_core::RobotModelPtr blend_model_;
  descartes_core::RobotModelPtr keyence_model_;
  moveit::core::RobotModelConstPtr moveit_model_;
  pluginlib::ClassLoader<descartes_core::RobotModel>
      plugin_loader_; // kept around so code doesn't get unloaded
  std::string blend_group_name_;
  std::string keyence_group_name_;
};
}

#endif
