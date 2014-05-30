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
#include "godel_process_path_generation/polygon_pts.hpp"


namespace godel_process_path
{
namespace polygon_utils
{

struct PolygonSegment
{
  PolygonSegment(const PolygonPt &p1, const PolygonPt &p2): start(p1), end(p2) {};
  PolygonPt start;
  PolygonPt end;

  /**@brief Does this segment intersect another
   * Does not account for segments that are within epsilon of touching, or nearly parallel */
  bool intersects(const PolygonSegment &other) const;
  inline double length() const {return start.dist(end);};
  inline PolygonPt vector() const {return end-start;};
};

/**@brief Checks that PolygonBoundary is valid (non-self-intersecting)
 * @param bnd Polygon to check
 * @param fast If true, do no intersection checks
 * @return True if polygon has no self-intersections
 */
bool checkBoundary(const PolygonBoundary &bnd, bool fast=false);

/**@brief Checks that PolygonBoundaryCollection is valid
 * Checks for self-intersection of each polygon, and global intersections of polygons
 * @param pbc PolygonBoundaryCollection to check
 * @return False if any intersection check is invalid, True otherwise
 */
bool checkBoundaryCollection(const PolygonBoundaryCollection &pbc);

/**@brief Find closest pt in PolygonBoundary to another pt on the plane
 * @param pt PolygonPt to compare against
 * @param bnd PolygonBoundary to find closest point in
 * @return pair(index into PolygonBoundary, distance)
 */
std::pair<size_t, float> closestPoint(const PolygonPt &pt, const PolygonBoundary &bnd);

} /* namespace polygon_utils */
} /* namespace godel_process_path */
#endif /* POLYGON_UTILS_H_ */
