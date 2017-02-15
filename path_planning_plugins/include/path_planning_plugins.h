#ifndef PATH_PLANNING_PLUGINS_H
#define PATH_PLANNING_PLUGINS_H

#include "path_planning_base.h"
#include <godel_process_path_generation/utils.h>
#include <godel_process_path_generation/polygon_utils.h>
#include <godel_process_path_generation/polygon_pts.hpp>
#include "profilometer_scan.h"
#include "mesh_importer.h"

namespace path_planning_plugins
{

namespace Openveronoi
{
  bool populatePlanningParameters(ros::NodeHandle& nh, path_planning_base::PlanningParams& params);

  class BlendPlanner : public path_planning_base::PathPlanningBase
  {
  private:
    pcl::PolygonMesh mesh_;
    path_planning_base::PlanningParams params_;
    mesh_importer::MeshImporter mesh_importer_;
    ros::NodeHandle nh_;
    ros::ServiceClient process_path_client_;

  public:
    BlendPlanner(){}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path);
  };

  class ScanPlanner : public path_planning_base::PathPlanningBase
  {
  private:
    pcl::PolygonMesh mesh_;
    path_planning_base::PlanningParams params_;
    mesh_importer::MeshImporter mesh_importer_;
    ros::NodeHandle nh_;

  public:
    ScanPlanner(){}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path);
  };
}
}

#endif // PATH_PLANNER_PLUGINS_H

