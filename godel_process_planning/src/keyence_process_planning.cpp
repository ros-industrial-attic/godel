#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

// descartes
#include "descartes_trajectory/cart_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "path_transitions.h"
#include "common_utils.h"
#include "generate_motion_plan.h"

namespace godel_process_planning
{

const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for robot state
/**
 * @brief Translated an Eigen pose to a Descartes trajectory point appropriate for the scan process!
 *        Mirros the function in blend_process_planning.cpp document.
 * @param pose
 * @param dt The upper limit of time from the previous point to achieve this one
 * @return A descartes trajectory point encapsulating a move to this pose
 */
descartes_core::TrajectoryPtPtr toDescartesScanPt(const Eigen::Affine3d& pose, double dt)
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
 * @brief Computes a joint motion plan based on input points and the scan process; this includes
 *        motion from current position to process path and back to the starting position.
 * @param req Process plan including reference pose, points, and process parameters
 * @param res Set of approach, process, and departure trajectories
 * @return True if a valid plan was generated; false otherwise
 */
bool ProcessPlanningManager::handleKeyencePlanning(godel_msgs::KeyenceProcessPlanning::Request& req,
                                                   godel_msgs::KeyenceProcessPlanning::Response& res)
{
  keyence_model_->setCheckCollisions(true);
  // Precondition: Input trajectory must be non-zero
  if (req.path.segments.empty())
  {
    ROS_WARN("%s: Cannot create scan process plan for empty trajectory", __FUNCTION__);
    return true;
  }

  if (req.path.segments.size() > 1)
  {
    ROS_WARN("%s: Currently we do not support scan paths w/ more than 1 segment."
             " Planning only for the first.", __FUNCTION__);
  }

  // Transform process path from geometry msgs to descartes points
  const static double LINEAR_DISCRETIZATION = 0.01; // meters
  const static double ANGULAR_DISCRETIZATION = 0.1; // radians
  const static double RETRACT_DISTANCE = 0.05; // meters

  TransitionParameters transition_params;
  transition_params.linear_disc = LINEAR_DISCRETIZATION;
  transition_params.angular_disc = ANGULAR_DISCRETIZATION;
  transition_params.retract_dist = RETRACT_DISTANCE;
  transition_params.traverse_height = req.params.approach_distance;
  transition_params.z_adjust = req.params.z_adjust;

  DescartesTraj process_points = toDescartesTraj(req.path.segments, req.params.traverse_spd, transition_params,
                                                 toDescartesScanPt);
  // Capture the current state of the robot
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  if (generateMotionPlan(keyence_model_, process_points, moveit_model_, keyence_group_name_,
                         current_joints, res.plan))
  {
    res.plan.type = res.plan.SCAN_TYPE;
    return true;
  }
  else
  {
    return false;
  }
}

}
