#ifndef PATH_PLANNER_PLUGINS_H
#define PATH_PLANNER_PLUGINS_H

#include <path_planner/path_planner_base.h>

namespace path_planner_plugins
{
  class OpenveronoiBlendPlanner : public path_planner_base::PathPlannerBase
  {
  private:
    pcl::PolygonMesh mesh_;
    godel_msgs::PathPlanningParameters params_;

  public:
    OpenveronoiBlendPlanner(){}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path, path_planner_base::PlanningParams params);
  };

  class OpenveronoiScanPlanner : public path_planner_base::PathPlannerBase
  {
  private:
    pcl::PolygonMesh mesh_;
    godel_msgs::PathPlanningParameters params_;

  public:
    OpenveronoiScanPlanner(){}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path, path_planner_base::PlanningParams params);
  };
}

#endif // PATH_PLANNER_PLUGINS_H
