#ifndef PATH_PLANNER_BASE_H_
#define PATH_PLANNER_BASE_H_

#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanningParameters.h>
#include <pcl/PolygonMesh.h>

namespace path_planner_base
{
  typedef  godel_msgs::PathPlanningParameters PlanningParams;
  class PathPlannerBase
  {
  public:
    virtual void init(pcl::PolygonMesh mesh) = 0;
    virtual bool generatePath(std::vector<geometry_msgs::PoseArray>& path,
                              godel_msgs::PathPlanningParameters params) = 0;
  };
}

#endif
