#include <path_planner/path_planner_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <godel_process_path_generation/process_path_generator.h>
#include <godel_process_path_generation/polygon_utils.h>
#include <services/surface_blending_service.h>

namespace path_planner_plugins
{

  void OpenveronoiPlanner::init(pcl::PolygonMesh mesh, PointCloud surface_cloud)
  {
    mesh_ = mesh;
    surface_cloud_ = surface_cloud;
  }

  bool OpenveronoiPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
  {
    ProcessPathResult result = SurfaceBlendingService::generateProcessPath()
  }
} // end mesher_plugins

PLUGINLIB_EXPORT_CLASS(path_planner_plugins::OpenveronoiPlanner, path_planner_base::PathPlannerBase)
