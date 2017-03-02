#include "path_transitions.h"

std::vector<godel_process_planning::ConnectingPath>
godel_process_planning::generateTransitions(const std::vector<geometry_msgs::PoseArray> &segments,
                                            const double traverse_height, const double linear_discretization)
{
  const int steps = std::ceil(traverse_height / linear_discretization);

  std::vector<ConnectingPath> result;

  // Loop over every connecting edge
  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    const auto& start_pose = segments[i].poses.front(); // First point in this segment
    const auto& end_pose = segments[i].poses.back();    // Last point in this segment

    Eigen::Affine3d e_start, e_end;
    tf::poseMsgToEigen(start_pose, e_start);
    tf::poseMsgToEigen(end_pose, e_end);

    // Each connecting segment has a retraction from 'from_pose'
    // And an approach to 'to_pose'
    auto approach = linearMoveZ(e_start, linear_discretization, steps);
    auto depart = linearMoveZ(e_end, linear_discretization, steps);
    std::reverse(approach.begin(), approach.end()); // we flip the 'to' path to keep the time ordering of the path

    ConnectingPath c;
    c.depart = std::move(depart);
    c.approach = std::move(approach);
    result.push_back(c);
  }
  return result;
}

using DescartesConversionFunc =
  boost::function<descartes_core::TrajectoryPtPtr (const Eigen::Affine3d &, const double)>;

godel_process_planning::DescartesTraj
godel_process_planning::toDescartesTraj(const std::vector<geometry_msgs::PoseArray> &segments,
                                        const double traverse_height, const double process_speed,
                                        const double linear_discretization, DescartesConversionFunc conversion_fn)
{
  auto transitions = generateTransitions(segments, traverse_height, linear_discretization);

  DescartesTraj traj;
  Eigen::Affine3d last_pose = createNominalTransform(segments.front().poses.front());

  // Convert pose arrays to Eigen types
  auto eigen_segments = toEigenArrays(segments);

  // Inline function for adding a sequence of motions
  auto add_segment = [&traj, &last_pose, process_speed, conversion_fn]
                     (const EigenSTL::vector_Affine3d& poses, bool free_last)
  {
    // Create Descartes trajectory for the segment path
    for (std::size_t j = 0; j < poses.size(); ++j)
    {
      Eigen::Affine3d this_pose = createNominalTransform(poses[j]);
      // O(1) jerky - may need to revisit this time parameterization later. This at least allows
      // Descartes to perform some optimizations in its graph serach.
      double dt = (this_pose.translation() - last_pose.translation()).norm() / process_speed;
      if (j == poses.size() - 1 && free_last)
      {
        dt = 0.0;
      }
      traj.push_back( conversion_fn(this_pose, dt) );
      last_pose = this_pose;
    }
  };

  for (std::size_t i = 0; i < segments.size(); ++i)
  {
    add_segment(transitions[i].approach, true);

    add_segment(eigen_segments[i], false);

    add_segment(transitions[i].depart, false);
  } // end segments

  return traj;
}
