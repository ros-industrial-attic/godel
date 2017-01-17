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
#ifndef PROFILIMITER_SCAN_H
#define PROFILIMITER_SCAN_H

#include <vector>
#include <godel_msgs/ScanPlanParameters.h>
#include <godel_process_path_generation/polygon_pts.hpp>

namespace godel_surface_detection
{
std::vector<godel_process_path::PolygonPt>
generateProfilimeterScanPath(const godel_process_path::PolygonBoundary& boundary,
                             const godel_msgs::ScanPlanParameters& params);

} // end namespace godel_surface_detection

#endif
