#include "generate_motion_plan.h"
#include "trajectory_utils.h"
#include "common_utils.h"
#include <descartes_trajectory/axial_symmetric_pt.h>
#include <descartes_trajectory/joint_trajectory_pt.h>

#include <descartes_planner/ladder_graph_dag_search.h>
#include <descartes_planner/dense_planner.h>

const static bool validateTrajectory(const trajectory_msgs::JointTrajectory& pts,
                                     const descartes_core::RobotModel& model,
                                     const double min_segment_size)
{
  for (std::size_t i = 1; i < pts.points.size(); ++i)
  {
    const auto& pt_a = pts.points[i - 1].positions;
    const auto& pt_b = pts.points[i].positions;

    auto interpolate = godel_process_planning::interpolateJoint(pt_a, pt_b, min_segment_size);

    // The thought here is that the graph building process already checks the waypoints in the
    // trajectory for collisions. What we want to do is check between these waypoints when they
    // move a lot. 'interpolateJoint()' returns a list of positions where the maximum joint motion
    // between them is no more 'than min_segment_size' and its inclusive. So if there are only two
    // solutions, then we just have the start & end which are already checked.
    if (interpolate.size() > 2)
    {
      for (std::size_t j = 1; j < interpolate.size() - 1; ++j)
      {
        if (!model.isValid(interpolate[j]))
        {
          return false;
        }
      }
    }
  }
  return true;
}

bool godel_process_planning::generateMotionPlan(const descartes_core::RobotModelPtr model,
                                                const std::vector<descartes_core::TrajectoryPtPtr> &traj,
                                                moveit::core::RobotModelConstPtr moveit_model,
                                                const std::string &move_group_name,
                                                const std::vector<double> &start_state,
                                                godel_msgs::ProcessPlan &plan)
{

  // Generate a graph of the process path joint solutions
  descartes_planner::PlanningGraph planning_graph (model);
  if (!planning_graph.insertGraph(traj)) // builds the graph out
  {
    ROS_ERROR("%s: Failed to build graph. One or more points may have no valid IK solutions", __FUNCTION__);
    return false;
  }

  // Using the valid starting configurations, let's compute an estimate
  // of the cost to move to these configurations from our starting pose
  const auto& graph = planning_graph.graph();
  const auto dof = graph.dof();

  std::vector<std::vector<double>> process_start_poses;
  const auto& joint_data = graph.getRung(0).data; // This is a flat vector of doubles w/ all the solutions
  const auto n_start_poses = joint_data.size() / dof;

  for (std::size_t i = 0; i < n_start_poses; ++i) // This builds a list of starting poses
  {
    std::vector<double> sol (&joint_data[i * dof], &joint_data[i*dof + dof]);
    process_start_poses.push_back(sol);
  }

  std::vector<double> process_start_costs (process_start_poses.size());
  for (std::size_t i = 0; i < process_start_costs.size(); ++i) // This computes the list of configurations
  {
    process_start_costs[i] = freeSpaceCostFunction(start_state, process_start_poses[i]);
  }

  // Now we perform the search using the starting costs from our estimation above
  descartes_planner::DAGSearch search (graph);
  double cost = search.run(process_start_costs);
  if (cost == std::numeric_limits<double>::max())
  {
    ROS_ERROR("%s: Failed to search graph. All points have IK, but process constraints (e.g velocity) "
              "prevent a solution", __FUNCTION__);
    return false;
  }

  // Here we search the graph for the shortest path and build a descartes trajectory of it!
  auto path_idxs = search.shortestPath();
  ROS_INFO("%s: Descartes computed path with cost %lf", __FUNCTION__, cost);
  DescartesTraj solution;
  for (size_t i = 0; i < path_idxs.size(); ++i)
  {
    const auto idx = path_idxs[i];
    const auto* data = graph.vertex(i, idx);
    const auto& tm = graph.getRung(i).timing;
    auto pt = descartes_core::TrajectoryPtPtr(new descartes_trajectory::JointTrajectoryPt(std::vector<double>(data, data + dof), tm));
    solution.push_back(pt);
  }

  // Now we plan our approach and depart to/from the path. We try to joint interpolate, and then we run from there
  try
  {
    trajectory_msgs::JointTrajectory approach =
        planFreeMove(*model, move_group_name, moveit_model,
                     start_state,
                     extractJoints(*model, *solution.front()));

    trajectory_msgs::JointTrajectory depart = planFreeMove(
        *model, move_group_name, moveit_model,
        extractJoints(*model, *solution.back()),
        start_state);

    // Break out the process path from the seed path and convert to ROS messages
    trajectory_msgs::JointTrajectory process = toROSTrajectory(solution, *model);

    const static double SMALLEST_VALID_SEGMENT = 0.05;
    if (!validateTrajectory(process, *model, SMALLEST_VALID_SEGMENT))
    {
      ROS_ERROR_STREAM("%s: Computed path contains joint configuration changes that would result in a collision.");
      return false;
    }

    // Fill in result trajectories
    plan.trajectory_process = process;
    plan.trajectory_approach = approach;
    plan.trajectory_depart = depart;

    // Fill in result header information
    const std::vector<std::string>& joint_names =
        moveit_model->getJointModelGroup(move_group_name)->getActiveJointModelNames();

    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_approach);
    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_depart);
    godel_process_planning::fillTrajectoryHeaders(joint_names, plan.trajectory_process);
    return true;
  }
  catch (const std::runtime_error& e)
  {
    ROS_ERROR_STREAM("Exception caught when planning blending path: " << e.what());
    return false;
  }
}
