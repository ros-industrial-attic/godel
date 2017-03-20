#ifndef GODEL_PROCESS_PLANNING_PATH_TRANSITIONS_H
#define GODEL_PROCESS_PLANNING_PATH_TRANSITIONS_H

#include "common_utils.h"
#include <godel_msgs/BlendingPlanParameters.h>
#include "eigen_conversions/eigen_msg.h"


namespace godel_process_planning
{

struct ConnectingPath
{
  EigenSTL::vector_Affine3d depart;
  EigenSTL::vector_Affine3d approach;
};

struct TransitionParameters
{
  double linear_disc;
  double angular_disc;
  double traverse_height;
  double retract_dist;
  double z_adjust;
};

std::vector<ConnectingPath> generateTransitions(const std::vector<geometry_msgs::PoseArray>& segments,
                                                const TransitionParameters& params);

/**
 * @brief transforms a sequence of pose-arrays, each representing a single 'segment' of a
 * process path into a Descartes specific format. This function also adds transitions between
 * segments.
 * @param segments Sequence of poses (relative to the world space of blending robot model)
 * @param traverse_height The height in meters from the surface of the part to move to before
 *        moving to the next segment start
 * @param process_speed The point to point difference in speed
 * @param linear_discretization The distance (meters) between points in the connecting paths
 * @param conversion_fn A function that creates a Descartes process point of whatever type your
 *        process (e.g. blending or scanning) requires
 * @return The input trajectory encoded in Descartes points
 */
godel_process_planning::DescartesTraj
toDescartesTraj(const std::vector<geometry_msgs::PoseArray>& segments,
                const double process_speed, const TransitionParameters& transition_params,
                boost::function<descartes_core::TrajectoryPtPtr(const Eigen::Affine3d&, const double)> conversion_fn);


}

#endif // GODEL_PROCESS_PLANNING_PATH_TRANSITIONS_H
