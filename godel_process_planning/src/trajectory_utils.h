#ifndef TRAJECTORY_UTILS_H
#define TRAJECTORY_UTILS_H

#include <Eigen/Geometry>

namespace godel_process_planning
{
// Cartesian Interpolation
typedef std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d> > PoseVector;

/**
   * @brief Creates a vector of poses representing linear spatial and rotational interpolation
   *        between starting and ending poses.
   * @param start Beginning pose
   * @param stop Terminating pose
   * @param ds The cartesian distance (m) between intermediate points
   * @return Sequence of poses
   */
PoseVector interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                                double ds);

// Joint Interpolation
typedef std::vector<std::vector<double> > JointVector;

/**
 * @brief Creates a vector of joint poses linearly interpolating from start to stop
 * @param start Initial joint configuration
 * @param stop Final joint configuration
 * @param dtheta Maximum joint step (radians) between intermediate points
 * @return
 */
JointVector interpolateJoint(const std::vector<double>& start, const std::vector<double>& stop,
                             double dtheta);
}

#endif
