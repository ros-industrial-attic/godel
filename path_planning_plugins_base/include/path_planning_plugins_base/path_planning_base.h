#ifndef PATH_PLANNING_PLUGINS_BASE_H_
#define PATH_PLANNING_PLUGINS_BASE_H_

#include <geometry_msgs/PoseArray.h>
#include <pcl/PolygonMesh.h>

namespace path_planning_plugins_base
{
  class PathPlanningBase
  {
  public:
    virtual void init(pcl::PolygonMesh mesh) = 0;
    virtual bool generatePath(std::vector<geometry_msgs::PoseArray>& path) = 0;
  };
}

#endif
