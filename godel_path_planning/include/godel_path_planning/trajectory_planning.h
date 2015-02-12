#ifndef GODEL_PATH_PLANNING_TRAJECTORY_PLANNING_H
#define GODEL_PATH_PLANNING_TRAJECTORY_PLANNING_H

#include "godel_msgs/TrajectoryPlanning.h"

namespace godel_path_planning
{
  bool generateTrajectory(const godel_msgs::TrajectoryPlanning::Request& req, 
                          trajectory_msgs::JointTrajectory& trajectory); // output parameter
}

#endif