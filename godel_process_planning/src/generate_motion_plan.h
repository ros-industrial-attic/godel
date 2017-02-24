#ifndef GODEL_GENERATE_MOTION_PLAN_H
#define GODEL_GENERATE_MOTION_PLAN_H

#include <descartes_core/robot_model.h>
#include <descartes_core/trajectory_pt.h>
#include <godel_msgs/ProcessPlan.h>

namespace godel_process_planning
{

/**
 * @brief This is a helper function for doing the joint level trajectory planning for a
 * a robot from a given \e start_state to and through the process path defined by \e
 * traj.
 * @param model A descartes robot model for this process/tool
 * @param traj A sequence of descartes points encapsulating the path tolerances
 * @param moveit_model A moveit robot model corresponding to the robot used
 * @param move_group_name The name of the move group being manipulated
 * @param start_state The initial position of the robot
 * @param plan Output parameter - the approach, process, and departure joint paths.
 * NOTE THAT ProcessPlan::type is NOT set.
 * @return True on planning success, false otherwise
 */
bool generateMotionPlan(const descartes_core::RobotModelPtr model,
                        const std::vector<descartes_core::TrajectoryPtPtr>& traj,
                        moveit::core::RobotModelConstPtr moveit_model,
                        const std::string& move_group_name,
                        const std::vector<double>& start_state,
                        godel_msgs::ProcessPlan& plan);


}

#endif // GODEL_GENERATE_MOTION_PLAN_H
