#include <path_planner/path_planner_plugins.h>
#include <pluginlib/class_list_macros.h>

namespace path_planner_plugins
{
  // Blend Planner

  void OpenveronoiBlendPlanner::init(pcl::PolygonMesh mesh)
  {
    mesh_ = mesh;
  }

  bool OpenveronoiBlendPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path,
                                             path_planner_base::PlanningParams params)
  {
    path.clear();
    return false;
  }


  // Scan Planner

  void OpenveronoiScanPlanner::init(pcl::PolygonMesh mesh)
  {
    mesh_ = mesh;
  }

  bool OpenveronoiScanPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path,
                                             path_planner_base::PlanningParams params)
  {
    path.clear();
    return false;
  }
} // end mesher_plugins

PLUGINLIB_EXPORT_CLASS(path_planner_plugins::OpenveronoiBlendPlanner, path_planner_base::PathPlannerBase)

PLUGINLIB_EXPORT_CLASS(path_planner_plugins::OpenveronoiScanPlanner, path_planner_base::PathPlannerBase)
