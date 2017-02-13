#include "generate_motion_plan.h"
#include "common_utils.h"
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

bool godel_process_planning::generateMotionPlan(const descartes_core::RobotModelPtr model,
                                                const std::vector<descartes_core::TrajectoryPtPtr> &traj,
                                                moveit::core::RobotModelConstPtr moveit_model,
                                                const std::string &move_group_name,
                                                const std::vector<double> &start_state,
                                                godel_msgs::ProcessPlan &plan)
{
  // Compute all of the joint poses at the start of the process path
  std::vector<std::vector<double> > start_joint_poses;
  traj.front()->getJointPoses(*model, start_joint_poses);

  if (start_joint_poses.empty())
  {
    ROS_WARN_STREAM("Blend Planning Service: Could not compute any inverse kinematic solutions for "
                    "the first point in the process path.");

    return false;
  }

  ROS_INFO_STREAM("Number of candidate poses: " << start_joint_poses.size());

  auto start_pose = godel_process_planning::pickBestStartPose(start_state, *model,
                                                              start_joint_poses,
                                                              freeSpaceCostFunction);

  DescartesTraj solved_path;
  // Calculate tool pose of robot starting config so that we can go back here on the
  // return move
  Eigen::Affine3d init_pose;
  model->getFK(start_state, init_pose);
  // Compute the nominal tool pose of the final process point
  Eigen::Affine3d process_stop_pose;
  traj.back()->getNominalCartPose(std::vector<double>(), *model,
                                  process_stop_pose);

  // Joint interpolate from the initial robot position to 'best' starting configuration of process
  // path
  DescartesTraj to_process = createJointPath(start_state, start_pose);
  to_process.front() =
      descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(start_state));

  // To get a rough estimate of process path cost, add a cartesian move from the final process point
  // to the starting position again.
  DescartesTraj from_process = createLinearPath(process_stop_pose, init_pose);
  from_process.back() =
      descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(start_state));

  // Affix the approach and depart paths calculated above with user process path
  DescartesTraj seed_path;
  seed_path.insert(seed_path.end(), to_process.begin(), to_process.end());
  seed_path.insert(seed_path.end(), traj.begin(), traj.end());
  seed_path.insert(seed_path.end(), from_process.begin(), from_process.end());

  // Attempt to solve the initial path (minimize joint motion over entire plan)
  if (!descartesSolve(seed_path, model, solved_path))
  {
    return false;
  }

  // Go back over and recalculate the approach and depart segments to:
  //  1. Use a joint interpolation from start to stop if collision free
  //  2. Use MoveIt (RRT-Connect) if the above fails
  // This is the only portion of the trajectory that is collision checked. Note this
  // method also converts the Descartes points into ROS trajectories.
  try
  {
    trajectory_msgs::JointTrajectory approach =
        planFreeMove(*model, move_group_name, moveit_model,
                     extractJoints(*model, *solved_path[0]),
                     extractJoints(*model, *solved_path[to_process.size()]));

    trajectory_msgs::JointTrajectory depart = planFreeMove(
        *model, move_group_name, moveit_model,
        extractJoints(*model, *solved_path[to_process.size() + traj.size() - 1]),
        extractJoints(*model, *solved_path[seed_path.size() - 1]));

    // Break out the process path from the seed path and convert to ROS messages
    DescartesTraj process_part(solved_path.begin() + to_process.size(),
                               solved_path.end() - from_process.size());
    trajectory_msgs::JointTrajectory process = toROSTrajectory(process_part, *model);

    for (std::size_t i = 0; i < process.points.size(); ++i)
    {
      const auto& pt = process.points[i];
      if (!model->isValid(pt.positions))
      {
        ROS_WARN_STREAM("Position in blending path (" << i << ") invalid: Joint limit or collision detected\n");
        return false;
      }
    }


    // Fill in result trajectories
    plan.trajectory_process = process;
    plan.trajectory_approach = approach;
    plan.trajectory_depart = depart;

    // Fill in result header information
    const std::vector<std::string>& joint_names =
        moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_approach);
    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_depart);
    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_process);
    return true;
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("Exception caught when planning blending path: " << e.what());
    return false;
  }
}
