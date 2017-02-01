#ifndef PATH_PLANNER_PLUGINS_H
#define PATH_PLANNER_PLUGINS_H

#include <path_planner/path_planner_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace path_planner_plugins
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  class OpenveronoiPlanner : public path_planner_base::PathPlannerBase
  {
  private:
    pcl::PolygonMesh mesh_;
    PointCloud surface_cloud_;

  public:
    OpenveronoiPlanner(){}
    void init(pcl::PolygonMesh, PointCloud);
    bool generatePath(std::vector<geometry_msgs::PoseArray>&);
  };
}

#endif // PATH_PLANNER_PLUGINS_H
