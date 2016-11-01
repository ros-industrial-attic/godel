#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

// descartes
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "common_utils.h"

// FILE LOCAL CONSTANTS
const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for robot state
const double FREE_SPACE_MAX_ANGLE_DELTA =
    M_PI_2; // The maximum angle a joint during a freespace motion
            // from the start to end position without that motion
            // being penalized. Avoids flips.
const double FREE_SPACE_ANGLE_PENALTY =
    5.0; // The factor by which a joint motion is multiplied if said
         // motion is greater than the max.

namespace godel_process_planning
{

/**
 * @brief Computes a 'cost' value for a robot motion between 'source' and 'target'
 * @param source  The joint configuration at start of motion
 * @param target  The joint configuration at end of motion
 * @return cost value
 */
 double freeSpaceCostFunctionScan(const std::vector<double>& source,
                                  const std::vector<double>& target)
{
  // The cost function here penalizes large single joint motions in an effort to
  // keep the robot from flipping a joint, even if some other joints have to move
  // a bit more.
  double cost = 0.0;
  for (std::size_t i = 0; i < source.size(); ++i)
  {
    double diff = std::abs(source[i] - target[i]);
    if (diff > FREE_SPACE_MAX_ANGLE_DELTA)
      cost += FREE_SPACE_ANGLE_PENALTY * diff;
    else
      cost += diff;
  }
  return cost;
}

/**
 * @brief Translated an Eigen pose to a Descartes trajectory point appropriate for the scan process!
 *        Mirros the function in blend_process_planning.cpp document.
 * @param pose
 * @param dt The upper limit of time from the previous point to achieve this one
 * @return A descartes trajectory point encapsulating a move to this pose
 */
static inline descartes_core::TrajectoryPtPtr toDescartesPt(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_trajectory;
  using namespace descartes_core;
  const descartes_core::TimingConstraint tm(dt);

  Eigen::Vector3d rpy = pose.rotation().eulerAngles(0, 1, 2);
  Eigen::Vector3d xyz = pose.translation();

  double rx = rpy(0), ry = rpy(1), rz = rpy(2);
  double x = xyz(0), y = xyz(1), z = xyz(2);

  // By specifying a range of 180 degrees and a step of 180 degrees, the system will sample
  // at the nominal pose +/- 90 degrees for scanning purposes.
  TolerancedFrame frame(
      pose, ToleranceBase::zeroTolerance<PositionTolerance>(x, y, z),
      ToleranceBase::createSymmetric<OrientationTolerance>(rx, ry, rz, 0, 0, M_PI));

  return TrajectoryPtPtr(new CartTrajectoryPt(frame, 0.0, M_PI, tm));
}

/**
 * @brief transforms an input, in the form of a reference pose and points relative to that pose,
 * into Descartes'
 *        native format. Also adds in associated parameters.
 * @param ref The reference posed that all points are multiplied by. Should be in the world space of
 * the keyence move group.
 * @param points Sequence of points (relative to ref and the world space of blending robot model)
 * @param params Surface blending parameters, including info such as traversal speed
 * @return The input trajectory encoded in Descartes points
 */
static godel_process_planning::DescartesTraj
toDescartesTraj(const geometry_msgs::Pose& ref, const std::vector<geometry_msgs::Point>& points,
                const godel_msgs::ScanPlanParameters& params)
{
  DescartesTraj traj;
  traj.reserve(points.size());
  if (points.empty())
    return traj;

  Eigen::Affine3d last_pose = createNominalTransform(ref, points.front());

  for (std::size_t i = 0; i < points.size(); ++i)
  {
    Eigen::Affine3d this_pose = createNominalTransform(ref, points[i]);
    double dt = (this_pose.translation() - last_pose.translation()).norm() / params.traverse_spd;
    traj.push_back(toDescartesPt(this_pose, dt));
    last_pose = this_pose;
  }

  return traj;
}

/**
 * @brief Computes a joint motion plan based on input points and the scan process; this includes
 *        motion from current position to process path and back to the starting position.
 * @param req Process plan including reference pose, points, and process parameters
 * @param res Set of approach, process, and departure trajectories
 * @return True if a valid plan was generated; false otherwise
 */
bool ProcessPlanningManager::handleKeyencePlanning(
    godel_msgs::KeyenceProcessPlanning::Request& req,
    godel_msgs::KeyenceProcessPlanning::Response& res)
{
  // Precondition: Input trajectory must be non-zero
  if (req.path.points.empty())
  {
    ROS_WARN("%s: Cannot create scan process plan for empty trajectory", __FUNCTION__);
    return false;
  }

  // Transform process path from geometry msgs to descartes points
  DescartesTraj process_points = toDescartesTraj(req.path.reference, req.path.points, req.params);

  // Capture the current state of the robot
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  // Compute all of the joint poses at the start of the process path
  std::vector<std::vector<double> > start_joint_poses;
  process_points.front()->getJointPoses(*keyence_model_, start_joint_poses);

  auto start_pose = pickBestStartPose(current_joints, *keyence_model_, start_joint_poses, freeSpaceCostFunctionScan);

  DescartesTraj solved_path;
  // Calculate tool pose of robot starting config so that we can go back here on the
  // return move
  Eigen::Affine3d init_pose;
  keyence_model_->getFK(current_joints, init_pose);
  // Compute the nominal tool pose of the final process point
  Eigen::Affine3d process_stop_pose;
  process_points.back()->getNominalCartPose(std::vector<double>(), *keyence_model_,
                                            process_stop_pose);

  // Joint interpolate from the initial robot position to 'best' starting configuration of process
  // path
  DescartesTraj to_process = createJointPath(current_joints, start_pose);
  to_process.front() =
      descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(current_joints));

  // To get a rough estimate of process path cost, add a cartesian move from the final process point
  // to the starting position again.
  DescartesTraj from_process = createLinearPath(process_stop_pose, init_pose);
  from_process.back() =
      descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(current_joints));

  // Stitch the above approach and departure trajectories in with the original path
  DescartesTraj seed_path;
  seed_path.insert(seed_path.end(), to_process.begin(), to_process.end());
  seed_path.insert(seed_path.end(), process_points.begin(), process_points.end());
  seed_path.insert(seed_path.end(), from_process.begin(), from_process.end());

  // Attempt to solve the initial path
  if (!descartesSolve(seed_path, keyence_model_, solved_path))
  {
    return false;
  }

  // Refine the approach and depart plans by attempting joint interpolation and using MoveIt if
  // necessary
  try
  {
    trajectory_msgs::JointTrajectory approach =
        planFreeMove(*keyence_model_, keyence_group_name_, moveit_model_,
                     extractJoints(*keyence_model_, *solved_path[0]),
                     extractJoints(*keyence_model_, *solved_path[to_process.size()]));

    trajectory_msgs::JointTrajectory depart = planFreeMove(
        *keyence_model_, keyence_group_name_, moveit_model_,
        extractJoints(*keyence_model_, *solved_path[to_process.size() + process_points.size() - 1]),
        extractJoints(*keyence_model_, *solved_path[seed_path.size() - 1]));
    // Segment out process path from initial solution
    DescartesTraj process_part(solved_path.begin() + to_process.size(),
                               solved_path.end() - from_process.size());
    trajectory_msgs::JointTrajectory process = toROSTrajectory(process_part, *keyence_model_);

    // Translate the Descartes trajectory into a ROS joint trajectory
    const std::vector<std::string>& joint_names =
        moveit_model_->getJointModelGroup(keyence_group_name_)->getActiveJointModelNames();

    res.plan.trajectory_process = process;
    res.plan.trajectory_approach = approach;
    res.plan.trajectory_depart = depart;

    godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_approach);
    godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_depart);
    godel_process_planning::fillTrajectoryHeaders(joint_names, res.plan.trajectory_process);

    res.plan.type = godel_msgs::ProcessPlan::SCAN_TYPE;

    return true;
  }
  catch (std::runtime_error& e)
  {
    return false;
  }
}
}
