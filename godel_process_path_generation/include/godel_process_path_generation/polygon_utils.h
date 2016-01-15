/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * polygon_utils.h
 *
 *  Created on: May 28, 2014
 *      Author: Dan Solomon
 */

#ifndef POLYGON_UTILS_H_
#define POLYGON_UTILS_H_

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "godel_process_path_generation/polygon_pts.hpp"

namespace godel_process_path
{
namespace polygon_utils
{

struct PolygonSegment
{
  PolygonSegment(const PolygonPt& p1, const PolygonPt& p2) : start(p1), end(p2){};
  PolygonPt start;
  PolygonPt end;

  /**@brief Does this segment intersect another
   * Does not account for segments that are nearly parallel
   * @param tol Tolerance on how close endpoints can be to line before considered touching */
  bool intersects(const PolygonSegment& other, double tol = 1e-5) const;
  inline double length() const { return start.dist(end); };
  inline double length2() const { return start.dist2(end); };
  inline PolygonPt vector() const { return end - start; };
  inline double cross(const PolygonSegment& other) const { return vector().cross(other.vector()); };
};

void boundaryToSegments(std::vector<PolygonSegment>& segments, const PolygonBoundary& polygon);

/**@brief Checks that PolygonBoundary is valid (non-self-intersecting)
 * @param bnd Polygon to check
 * @return True if polygon has no self-intersections
 */
bool checkBoundary(const PolygonBoundary& bnd);

/**@brief Checks that PolygonBoundaryCollection is valid
 * Checks for self-intersection of each polygon, and global intersections of polygons
 * @param pbc PolygonBoundaryCollection to check
 * @return False if any intersection check is invalid, True otherwise
 */
bool checkBoundaryCollection(const PolygonBoundaryCollection& pbc);

/**@brief calculate total length of boundary by summing segments */
inline double circumference(const PolygonBoundary& boundary)
{
  double dist(0); // Total distance squared
  for (PolygonBoundary::const_iterator pt = boundary.begin(); pt != boost::prior(boundary.end());
       ++pt)
  {
    dist += pt->dist(*boost::next(pt));
  }
  dist += boundary.back().dist(boundary.front());
  return dist;
}

/**@brief Find closest pt in PolygonBoundary to another pt on the plane
 * @param pt PolygonPt to compare against
 * @param bnd PolygonBoundary to find closest point in
 * @return pair(index into PolygonBoundary, distance)
 */
std::pair<size_t, float> closestPoint(const PolygonPt& pt, const PolygonBoundary& bnd);

/**@brief Downsample boundary so that edges are represented by endpoints
 * Points may be removed from boundary to satisfy this filter
 * @param boundary PolygonBoundary to modify
 * @param tol allowable angle between adjacent segments to decide if edge is linear. If tol is
 * outside [0, pi/2] it is reset to closest bound.
 */
void filter(PolygonBoundary& boundary, double tol);

/**@brief Downsample all boundaries so that edges are represented by endpoints
 * Points may be removed from boundaries to satisfy this filter
 * @param boundaries PolygonBoundaryCollection to modify
 * @param tol allowable angle between adjacent segments to decide if edge is linear. If tol is
 * outside [0, pi/2] it is reset to closest bound.
 */
inline void filter(PolygonBoundaryCollection& boundaries, double tol)
{
  // Check bounds of tol.
  if (tol > M_PI_2)
  {
    ROS_WARN("Global filter tolerance set above pi/2 (%lf), using %f", tol, M_PI_2);
    tol = M_PI_2;
  }
  else if (tol < 0.)
  {
    ROS_WARN("Global filter tolerance set below 0 (%lf), using 0", tol);
    tol = 0.;
  }

  BOOST_FOREACH (PolygonBoundary& boundary, boundaries)
  {
    filter(boundary, tol);
  }
}

/**@brief Check if a intersects b */
bool intersects(const PolygonBoundary& a, const PolygonBoundary& b);

/**@brief Check if a intersects b */
bool intersects(const std::vector<PolygonSegment>& a, const std::vector<PolygonSegment>& b);

} /* namespace polygon_utils */
} /* namespace godel_process_path */
#endif /* POLYGON_UTILS_H_ */
