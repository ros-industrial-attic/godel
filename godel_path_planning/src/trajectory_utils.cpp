#include "godel_path_planning/trajectory_utils.h"


// Translates descartes trajectory points to ROS trajectory Ppoints
void godel_path_planning::populateTrajectoryMsg(const TrajectoryVec& solution,
                                                const descartes_core::RobotModel& robot_model,
                                                trajectory_msgs::JointTrajectory& trajectory)
{
  typedef std::vector<descartes_core::TrajectoryPtPtr>::const_iterator JointSolutionIterator;
  const static double DEFAULT_TIME = 1.0;
  
  ros::Duration time_from_start (0.0);
  for (JointSolutionIterator it = solution.begin(); it != solution.end(); ++it)
  {
    // Retrieve actual target joint angles from the polymorphic interface function
    std::vector<double> sol;
    it->get()->getNominalJointPose(std::vector<double>(), robot_model, sol);
    
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions = sol;
    point.velocities.resize(sol.size(), 0.0); // Fill extra fields with zeroes for now
    point.accelerations.resize(sol.size(), 0.0);
    point.effort.resize(sol.size(), 0.0);

    double time_step = it->get()->getTiming().upper_;
    if (time_step == 0.0)
    {
      time_from_start += ros::Duration(DEFAULT_TIME);
    }
    else
    {
      time_from_start += ros::Duration(time_step);
    }

    point.time_from_start = time_from_start;

    // add trajectory point to array
    trajectory.points.push_back(point);
  }
  return;
}