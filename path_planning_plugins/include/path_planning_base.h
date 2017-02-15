#ifndef PATH_PLANNING_BASE_H_
#define PATH_PLANNING_BASE_H_

#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanningParameters.h>
#include <pcl/PolygonMesh.h>

namespace path_planning_base
{
  typedef  godel_msgs::PathPlanningParameters PlanningParams;
  class PathPlanningBase
  {
  public:
    virtual void init(pcl::PolygonMesh mesh) = 0;
    virtual bool generatePath(std::vector<geometry_msgs::PoseArray>& path) = 0;
  };
}

#endif
