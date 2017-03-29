#include "trajectory_utils.h"

//////////////////////////////////////////////
// Helper Functions for joint interpolation //
//////////////////////////////////////////////

static unsigned int calculateRequiredSteps(const std::vector<double>& start,
                                           const std::vector<double>& stop, double dtheta)
{
  // calculate max steps required
  unsigned steps = 0;
  for (std::size_t i = 0; i < start.size(); ++i)
  {
    unsigned this_joint_steps = static_cast<unsigned>(std::ceil(std::abs(start[i] - stop[i]) / dtheta));
    steps = std::max(steps, this_joint_steps);
  }

  return steps;
}

static std::vector<double> calculateJointSteps(const std::vector<double>& start,
                                               const std::vector<double>& stop, unsigned int steps)
{
  // Given max steps, calculate delta for each joint
  std::vector<double> result;
  result.reserve(start.size());

  for (std::size_t i = 0; i < start.size(); ++i)
  {
    result.push_back((stop[i] - start[i]) / steps);
  }

  return result;
}

static std::vector<double> interpolateJointSteps(const std::vector<double>& start,
                                                 const std::vector<double>& step_size,
                                                 unsigned step)
{
  std::vector<double> result;
  result.reserve(start.size());
  for (std::size_t i = 0; i < start.size(); ++i)
    result.push_back(start[i] + step_size[i] * step);
  return result;
}

godel_process_planning::PoseVector
godel_process_planning::interpolateCartesian(const Eigen::Affine3d& start,
                                             const Eigen::Affine3d& stop, double ds)
{
  // Required position change
  Eigen::Vector3d delta = (stop.translation() - start.translation());
  Eigen::Vector3d start_pos = start.translation();

  // Calculate number of steps
  unsigned steps = static_cast<unsigned>(delta.norm() / ds) + 1;

  // Step size
  Eigen::Vector3d step = delta / steps;

  // Orientation interpolation
  Eigen::Quaterniond start_q(start.rotation());
  Eigen::Quaterniond stop_q(stop.rotation());
  double slerp_ratio = 1.0 / steps;

  godel_process_planning::PoseVector result;
  result.reserve(steps);
  for (unsigned i = 0; i <= steps; ++i)
  {
    Eigen::Vector3d trans = start_pos + step * i;
    Eigen::Quaterniond q = start_q.slerp(slerp_ratio * i, stop_q);
    Eigen::Affine3d pose(Eigen::Translation3d(trans) * q);
    result.push_back(pose);
  }
  return result;
}

godel_process_planning::JointVector
godel_process_planning::interpolateJoint(const std::vector<double>& start,
                                         const std::vector<double>& stop, double dtheta)
{
  godel_process_planning::JointVector result;
  // joint delta
  unsigned steps = calculateRequiredSteps(start, stop, dtheta);
  std::vector<double> delta = calculateJointSteps(start, stop, steps);
  // Walk interpolation
  for (std::size_t i = 0; i <= steps; ++i)
  {
    std::vector<double> pos = interpolateJointSteps(start, delta, i);
    result.push_back(pos);
  }
  return result;
}
