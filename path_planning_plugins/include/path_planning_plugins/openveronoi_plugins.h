#ifndef OPENVERONOI_PLUGINS_H
#define OPENVERONOI_PLUGINS_H

#include <path_planning_plugins_base/path_planning_base.h>
#include <godel_process_path_generation/utils.h>
#include <godel_process_path_generation/polygon_utils.h>
#include <godel_process_path_generation/polygon_pts.hpp>

const static std::string DEFAULT_PARAM_PREFIX = "/path_planning_params/";
const static std::string DISCRETIZATION = DEFAULT_PARAM_PREFIX + "discretization";
const static std::string MARGIN = DEFAULT_PARAM_PREFIX + "margin";
const static std::string OVERLAP = DEFAULT_PARAM_PREFIX + "overlap";
const static std::string SAFE_TRAVERSE_HEIGHT = DEFAULT_PARAM_PREFIX + "safe_traverse_height";
const static std::string SCAN_WIDTH = DEFAULT_PARAM_PREFIX + "scan_width";
const static std::string TOOL_RADIUS = DEFAULT_PARAM_PREFIX + "tool_radius";
const static double MIN_BOUNDARY_LENGTH = 0.1; // 10 cm

namespace path_planning_plugins
{
namespace openveronoi
{
  static godel_process_path::PolygonBoundaryCollection
  filterPolygonBoundaries(const godel_process_path::PolygonBoundaryCollection& boundaries)
  {
    godel_process_path::PolygonBoundaryCollection filtered_boundaries;

    for (std::size_t i = 0; i < boundaries.size(); ++i)
    {
      const godel_process_path::PolygonBoundary& bnd = boundaries[i];
      double circ = godel_process_path::polygon_utils::circumference(bnd);

      if (circ < MIN_BOUNDARY_LENGTH)
      {
        ROS_WARN_STREAM("Ignoring boundary with length " << circ);
      }
      else if (!godel_process_path::polygon_utils::checkBoundary(bnd))
      {
        ROS_WARN_STREAM("Ignoring ill-formed boundary");
      }
      else
      {
        filtered_boundaries.push_back(bnd);
        godel_process_path::polygon_utils::filter(filtered_boundaries.back(), 0.1);
        std::reverse(filtered_boundaries.back().begin(), filtered_boundaries.back().end());
      }
    }
    return filtered_boundaries;
  }


  class BlendPlanner : public path_planning_plugins_base::PathPlanningBase
  {
  private:
    pcl::PolygonMesh mesh_;    

  public:
    BlendPlanner() {}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path);
  };

  class ScanPlanner : public path_planning_plugins_base::PathPlanningBase
  {
  private:
    pcl::PolygonMesh mesh_;

  public:
    ScanPlanner() {}
    void init(pcl::PolygonMesh mesh);
    bool generatePath(std::vector<geometry_msgs::PoseArray>& path);
  };
}
}

#endif // OPENVERONOI_PLUGINS_H

