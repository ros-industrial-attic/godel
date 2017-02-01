#ifndef PATH_PLANNER_BASE_H_
#define PATH_PLANNER_BASE_H_

#include <geometry_msgs/PoseArray.h>
#include <pcl/PolygonMesh.h>

namespace path_planner_base
{
  class PathPlannerBase
  {
  public:
    virtual void init(pcl::PolygonMesh) = 0;
    virtual bool generatePath(std::vector<geometry_msgs::PoseArray>&) = 0;
  };
}

#endif
