#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

// descartes
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "common_utils.h"
#include "path_transitions.h"
#include "generate_motion_plan.h"
#include "boost/make_shared.hpp"

namespace godel_process_planning
{

// Planning Constants
const double BLENDING_ANGLE_DISCRETIZATION =
    M_PI / 12.0; // The discretization of the tool's pose about
                 // the z axis
const static std::string JOINT_TOPIC_NAME =
    "joint_states"; // ROS topic to subscribe to for current robot
                    // state info
/**
 * @brief Translated an Eigen pose to a Descartes trajectory point appropriate for the BLEND
 * process!
 *        Note that this function is local only to this file, and there is a similar function in
 *        the keyence_process_planning.cpp document.
 * @param pose
 * @param dt The upper limit of time from the previous point to achieve this one
 * @return A descartes trajectory point encapsulating a move to this pose
 */
descartes_core::TrajectoryPtPtr toDescartesBlendPt(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_trajectory;
  using namespace descartes_core;
  const TimingConstraint tm(dt);
  return boost::make_shared<AxialSymmetricPt>(pose, BLENDING_ANGLE_DISCRETIZATION,
                                              AxialSymmetricPt::Z_AXIS, tm);
}

/**
 * @brief Computes a joint motion plan based on input points and the blending process; this includes
 *        motion from current position to process path and back to the starting position.
 * @param req Process plan including reference pose, points, and process parameters
 * @param res Set of approach, process, and departure trajectories
 * @return True if a valid plan was generated; false otherwise
 */
bool ProcessPlanningManager::handleBlendPlanning(godel_msgs::BlendProcessPlanning::Request& req,
                                                 godel_msgs::BlendProcessPlanning::Response& res)
{
  // Enable Collision Checks
  blend_model_->setCheckCollisions(true);

  // Precondition: There must be at least one input segments
  if (req.path.segments.empty())
  {
    ROS_WARN("Planning request contained no trajectory segments. Nothing to be done.");
    return true;
  }

  // Precondition: All input segments must have at least one pose associated with them
  for (const auto& segment : req.path.segments)
  {
    if (segment.poses.empty())
    {
      ROS_ERROR("Input trajectory segment contained no poses. Invalid input.");
      return false;
    }
  }

  // Transform process path from geometry msgs to descartes points
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  const static double LINEAR_DISCRETIZATION = 0.01; // meters
  const static double ANGULAR_DISCRETIZATION = 0.1; // radians
  const static double RETRACT_DISTANCE = 0.05; // meters

  TransitionParameters transition_params;
  transition_params.linear_disc = LINEAR_DISCRETIZATION;
  transition_params.angular_disc = ANGULAR_DISCRETIZATION;
  transition_params.retract_dist = RETRACT_DISTANCE;
  transition_params.traverse_height = req.params.safe_traverse_height;
  transition_params.z_adjust = req.params.z_adjust;

  DescartesTraj process_points = toDescartesTraj(req.path.segments, req.params.traverse_spd, transition_params,
                                                 toDescartesBlendPt);

  if (generateMotionPlan(blend_model_, process_points, moveit_model_, blend_group_name_,
                         current_joints, res.plan))
  {
    res.plan.type = res.plan.BLEND_TYPE;
    return true;
  }
  else
  {
    return false;
  }
}

} // end namespace
