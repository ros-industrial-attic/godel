#ifndef TRAJECTORY_INTERPOLATOR_H
#define TRAJECTORY_INTERPOLATOR_H

#include <godel_path_planning/trajectory_utils.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

namespace godel_path_planning
{

  TrajectoryVec 
  interpolateCartesian(const Eigen::Affine3d& start, const Eigen::Affine3d& stop,
                       double total_time, double ds);

  TrajectoryVec
  jointInterpolate(const std::vector<double>& start, const std::vector<double>& stop,
                   double total_time, unsigned steps);

  inline
  bool hasJointJump(const std::vector<double>& from, const std::vector<double>& to, double thresh)
  {
      double sum = 0.0;
      for (std::size_t i = 0; i < from.size(); ++i)
          sum += std::abs(to[i] - from[i]);
      return sum > thresh;
  }

}

#endif
