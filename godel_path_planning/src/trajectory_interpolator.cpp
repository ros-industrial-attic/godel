#include "godel_path_planning/trajectory_interpolator.h"


std::vector<descartes_core::TrajectoryPtPtr> 
godel_path_planning::interpolateCartesian(const Eigen::Affine3d& start, 
                                          const Eigen::Affine3d& stop,
                                          double total_time, double ds)
{
  // Required position change
  Eigen::Vector3d delta = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();
  // Calculate number of steps
  unsigned steps = static_cast<unsigned>(delta.norm() / ds) + 1;
  // Step size
  Eigen::Vector3d step = delta / steps;
  // Time delta
  descartes_core::TimingConstraint timing (0, 0.2);//total_time/steps);
  // Orientation interpolation
  Eigen::Quaterniond start_q (start.rotation());
  Eigen::Quaterniond stop_q (stop.rotation());
  double slerp_ratio = 1.0 / steps;
  // Tolerances
  const Eigen::Vector3d pos_tol (0, 0, 0);
  const Eigen::Vector3d orient_tol(M_PI, 0, 2*M_PI);

  std::vector<descartes_core::TrajectoryPtPtr> result;
  result.reserve(steps);
  for (unsigned i = 0; i <= steps; ++i)
  {
    Eigen::Vector3d trans = start_pos + step * i;
    Eigen::Quaterniond q = start_q.slerp(slerp_ratio * i, stop_q);
    Eigen::Affine3d pose (Eigen::Translation3d(trans) * q);
    auto ptr = makeCartesianPt(pose, pos_tol, orient_tol, 0, M_PI/6, timing);
    result.push_back(ptr);
  }
  return result;
}

//////////////////////////////////////////////
// Helper Functions for joint interpolation //
//////////////////////////////////////////////

static std::vector<double> calculateJointSteps(const std::vector<double>& start,
                                               const std::vector<double>& stop,
                                               unsigned steps)
{
    std::vector<double> result;
    result.reserve(start.size());
    for (std::size_t i = 0; i < start.size(); ++i)
        result.push_back( (stop[i] - start[i]) / steps );
    return result;
}

static std::vector<double> interpolateJointSteps(const std::vector<double>& start,
                                                 const std::vector<double>& step_size,
                                                 unsigned step)
{
    std::vector<double> result;
    result.reserve(start.size());
    for (std::size_t i = 0; i < start.size(); ++i)
        result.push_back( start[i] + step_size[i] * step );
    return result;
}

std::vector<descartes_core::TrajectoryPtPtr>
godel_path_planning::jointInterpolate(const std::vector<double>& start, 
                                      const std::vector<double>& stop,
                                      double total_time, unsigned steps)
{
  std::vector<descartes_core::TrajectoryPtPtr> result;
  // joint delta
  std::vector<double> delta = calculateJointSteps(start, stop, steps);
  // time delta
  descartes_core::TimingConstraint timing (0.0, total_time/steps);
  // Walk interpolation
  for (std::size_t i = 0; i <= steps; ++i)
  {
      std::vector<double> pos = interpolateJointSteps(start, delta, i);
      descartes_core::TrajectoryPtPtr ptr (new descartes_trajectory::JointTrajectoryPt(pos, timing));
      result.push_back(ptr);
  }
  return result;
}
