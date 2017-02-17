/**
 * This defines some utilities for generating keyence scan paths from a
 * polygon boundary.
 *
 * The input is a PolygonBoundary which defines the 2d points comprising
 * a hole-less surface
 *
 * The output is a set of dense points which, when packaged with the associated
 * pose of the surface, are suitable for trajectory planning.
 */
#ifndef PROFILOMETER_SCAN_H
#define PROFILOMETER_SCAN_H

#include <godel_process_path_generation/polygon_pts.hpp>
#include <godel_msgs/PathPlanningParameters.h>
namespace path_planning_plugins
{
namespace scan
{

std::vector<godel_process_path::PolygonPt>
generateProfilometerScanPath(const godel_process_path::PolygonBoundary& boundary,
                             const godel_msgs::PathPlanningParameters& params);

} // end namespace scan
} // end namespace path_planning_plugins

#endif
