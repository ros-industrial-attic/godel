#include <godel_process_planning/godel_process_planning.h>

#include <ros/console.h>

// descartes
#include "descartes_trajectory/axial_symmetric_pt.h"
#include "descartes_trajectory/joint_trajectory_pt.h"
#include "descartes_planner/dense_planner.h"

#include "common_utils.h"
#include "generate_motion_plan.h"
#include "eigen_conversions/eigen_msg.h"
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
static inline descartes_core::TrajectoryPtPtr toDescartesPt(const Eigen::Affine3d& pose, double dt)
{
  using namespace descartes_trajectory;
  using namespace descartes_core;
  const TimingConstraint tm(dt);
  return boost::make_shared<AxialSymmetricPt>(pose, BLENDING_ANGLE_DISCRETIZATION,
                                              AxialSymmetricPt::Z_AXIS, tm);
}

static EigenSTL::vector_Affine3d linearMoveZ(const Eigen::Affine3d& origin, double step_size, int steps)
{
  EigenSTL::vector_Affine3d result (steps);

  for (int i = 0; i < steps; ++i)
  {
    result[i] = origin * Eigen::Translation3d(0, 0, step_size * (i + 1));
  }

  return result;
}



/**
 * @brief toDescartesTraj
 * @param poses
 * @param params
 * @return
 */
static godel_process_planning::DescartesTraj
toDescartesTraj(const std::vector<geometry_msgs::PoseArray>& segments,
                const godel_msgs::BlendingPlanParameters& params)
{
  std::vector<EigenSTL::vector_Affine3d> connections;

  const static double step_size = 0.02; // m

  int steps = std::ceil(params.safe_traverse_height / step_size);

  // Loop over every connecting edge
  for (std::size_t i = 1; i < segments.size(); ++i)
  {
    const auto& from_pose = segments[i - 1].poses.back(); // Where we come from...
    const auto& to_pose = segments[i].poses.front();      // Where we want to end up

    Eigen::Affine3d e_from, e_to;
    tf::poseMsgToEigen(from_pose, e_from);
    tf::poseMsgToEigen(to_pose, e_to);

    // Each connecting segment has a retraction from 'from_pose'
    // And an approach to 'to_pose'
    auto from = linearMoveZ(e_from, step_size, steps);
    auto to = linearMoveZ(e_to, step_size, steps);
    std::reverse(to.begin(), to.end()); // we flip the 'to' path to keep the time ordering of the path

    connections.push_back(from);
    connections.push_back(to);
  }

  DescartesTraj traj;
  Eigen::Affine3d last_pose = createNominalTransform(segments.front().poses.front());

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    // Create Descartes trajectory for the segment path
    for (std::size_t j = 0; j < segments[i].poses.size(); ++j)
    {
      Eigen::Affine3d this_pose = createNominalTransform(segments[i].poses[j]);
      // O(1) jerky - may need to revisit this time parameterization later. This at least allows
      // Descartes to perform some optimizations in its graph serach.
      double dt = (this_pose.translation() - last_pose.translation()).norm() / params.traverse_spd;
      traj.push_back(toDescartesPt(this_pose, dt));
      last_pose = this_pose;
    }

    // If we're not on the last segment, then we have connections to add
    if (i != segments.size() - 1)
    {
      const auto& depart = connections[i * 2 + 0];
      const auto& approach = connections[i * 2 + 1];

      // Create Descartes trajectory for the departure path
      for (std::size_t j = 0; j < depart.size(); ++j)
      {
        Eigen::Affine3d this_pose = createNominalTransform(depart[j]);
        // O(1) jerky - may need to revisit this time parameterization later. This at least allows
        // Descartes to perform some optimizations in its graph serach.
        double dt = (this_pose.translation() - last_pose.translation()).norm() / params.traverse_spd;
        traj.push_back(toDescartesPt(this_pose, dt));
        last_pose = this_pose;
      }

      for (std::size_t j = 0; j < approach.size(); ++j)
      {
        Eigen::Affine3d this_pose = createNominalTransform(approach[j]);
        // O(1) jerky - may need to revisit this time parameterization later. This at least allows
        // Descartes to perform some optimizations in its graph serach.
        double dt =
            j == 0 ? 0.0 : (this_pose.translation() - last_pose.translation()).norm() / params.traverse_spd;
        traj.push_back(toDescartesPt(this_pose, dt));
        last_pose = this_pose;
      }
    } // end connections
  } // end segments

  return traj;
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
    return false;
  }

  // Precondition: All input segments must have at least one pose associated with them
  for (const auto& segment : req.path.segments)
  {
    if (segment.poses.empty())
    {
      ROS_WARN("Input trajectory segment contained no poses. Invalid input.");
      return false;
    }
  }

  // Transform process path from geometry msgs to descartes points
  std::vector<double> current_joints = getCurrentJointState(JOINT_TOPIC_NAME);

  DescartesTraj process_points = toDescartesTraj(req.path.segments, req.params);

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
