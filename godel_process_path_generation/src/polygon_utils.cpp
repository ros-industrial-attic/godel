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
 * polygon_utils.cpp
 *
 *  Created on: May 28, 2014
 *      Author: ros
 */

#include "godel_process_path_generation/polygon_utils.h"
#include <boost/next_prior.hpp>
#include <boost/foreach.hpp>
#include <Eigen/Geometry>

namespace godel_process_path
{
namespace polygon_utils
{

bool PolygonSegment::intersects(const PolygonSegment &other) const
{
  /* Two vectors: intersection pt = p1 + u(p2-p1) = p3 + v(p3-p1)
   * Cramer's Rule to solve for u,v and check if they are both in bounds of segments.
   */
  double denom = (this->vector()).cross(other.vector());
  if (denom < 1e-12)
  {
    return false;  // Lines are (nearly) parallel
  }

  // Solve for u
  PolygonPt del = other.start - this->start;
  double u = del.cross(other.vector()) / denom;
  if (u < 0. || u > length())
  { // Intersection is outside of segment 1
    return false;
  }
  else
  { // Possible intersection, must check segment 2
    double v = this->vector().cross(del) / denom;
    if (v < 0. || v > other.length())
    {
      return false;
    }
  }
  return true;
}

bool checkBoundary(const PolygonBoundary &bnd, bool fast)
{
  const size_t N = bnd.size();
  if (N < 3)
  {
    return false;
  }

  // Check for 0-length segments
  const double LENGTH_TOL = 100.*std::numeric_limits<double>::epsilon();
  PolygonBoundary::const_iterator pt, last_pt;
  for (pt=bnd.begin(), last_pt=boost::prior(bnd.end()); pt!=bnd.end(); ++pt)
  {
    if (pt==last_pt)
    {
      if (pt->dist(bnd.front()) < LENGTH_TOL)
      {
        return false;
      }
    }
    else if (pt->dist(*boost::next(pt)) < LENGTH_TOL)
    {
      return false;
    }
  }

  // Subsequent checks for intersection are pointless on a triangle
  if (N == 3)
  {
    return true;
  }

  // Return early if fast check is requested (no intersection check)
  if (fast)
  {
    return true;
  }

  // Represent entire boundary as segments
  std::vector<PolygonSegment> segments;

  for (pt=bnd.begin(); pt!=bnd.end(); ++pt)
  {
    if (pt == last_pt)
    {
      segments.push_back(PolygonSegment(*pt, bnd.front()));
    }
    else
    {
      segments.push_back(PolygonSegment(*pt, *boost::next(pt)));
    }
  }
  for (std::vector<PolygonSegment>::const_iterator seg=segments.begin(); seg!=(segments.end()-2); ++seg)
  {
    for (std::vector<PolygonSegment>::const_iterator other_seg=seg+2; other_seg!=segments.end(); ++other_seg)
    {
      if (seg->intersects(*other_seg))
      {
        return false;
      }
    }
  }

  return true;
}

bool checkBoundaryCollection(const PolygonBoundaryCollection &pbc)
{
  // Check everything except self-intersection for each boundary
  // Also add each boundary to segments
  bool fast_check = true;
  std::vector<PolygonSegment> segments;
  BOOST_FOREACH(const PolygonBoundary &bnd, pbc)
  {
    if (!checkBoundary(bnd, fast_check)) /*call check boundary without intersection checks*/
    {
      return false;
    }
    BOOST_FOREACH(const PolygonPt &pt, bnd)
    {
      if (&pt == &bnd.back())
      {
        segments.push_back(PolygonSegment(pt, bnd.front()));
      }
      else
      {
        std::vector<PolygonPt>::const_iterator pt_iter(&pt);
        segments.push_back(PolygonSegment(pt, *boost::next(pt_iter)));
      }
    }
  }
  std::cout << "Created segments" << std::endl;

  // Check global intersections between boundaries
  //TODO this can miss intersections between the end of one polygon and the beginning of the next. Truly need to check each polygon against every other polygon
  for (std::vector<PolygonSegment>::const_iterator seg=segments.begin(); seg!=(segments.end()-2); ++seg)
  {
    for (std::vector<PolygonSegment>::const_iterator other_seg=seg+2; other_seg!=segments.end(); ++other_seg)
    {
      if (seg->intersects(*other_seg))
      {
        return false;
      }
    }
  }

  return true;
}

std::pair<size_t, float> closestPoint(const PolygonPt &pt, const PolygonBoundary &bnd)
{
  size_t point_num;
  double close_dist2(std::numeric_limits<double>::max());
  for (size_t idx=0, max=bnd.size()-1; idx != max; ++idx )
  {
    double prox2 = pt.dist2(bnd.at(idx));
    if (prox2 < close_dist2)
    {
      close_dist2 = prox2;
      point_num = idx;
    }
  }
  return std::make_pair(point_num, std::sqrt(static_cast<float>(close_dist2)));
}

} /* namespace polygon_utils */
} /* namespace godel_process_path */
