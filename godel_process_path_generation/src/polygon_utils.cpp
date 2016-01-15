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

bool PolygonSegment::intersects(const PolygonSegment& other, double tol) const
{
  /* Two vectors: intersection pt = p1 + u(p2-p1) = p3 + v(p4-p3)
   * u(p2-p1) + v(-p4+p3) = (p3-p1) = del
   * u = (del) X (-p4+p3) / (p2-p1)X(-p4+p3); v = (p2-p1) X (del) / (p2-p1)X(-p4+p3)
   * Cramer's Rule to solve for u,v and check if they are both in bounds of segments.
   */
  double denom = -cross(other);
  if (std::abs(denom) < 1e-12)
  {
    return false; // Lines are (nearly) parallel
  }

  // Solve for u
  PolygonPt del = other.start - this->start;
  double u = -del.cross(other.vector()) / denom;
  if (u < 0. - tol || u > 1. + tol)
  { // Intersection is outside of segment 1
    return false;
  }
  else
  { // Possible intersection, must check segment 2
    double v = this->vector().cross(del) / denom;
    if (v < 0. - tol || v > 1. + tol)
    {
      return false;
    }
  }
  return true;
}

void boundaryToSegments(std::vector<PolygonSegment>& segments, const PolygonBoundary& polygon)
{
  segments.clear();
  PolygonBoundary::const_iterator pt, last_pt;
  for (pt = polygon.begin(), last_pt = boost::prior(polygon.end()); pt != polygon.end(); ++pt)
  {
    if (pt == last_pt)
    {
      segments.push_back(PolygonSegment(*pt, polygon.front()));
    }
    else
    {
      segments.push_back(PolygonSegment(*pt, *boost::next(pt)));
    }
  }
}

void filter(PolygonBoundary& boundary, double tol)
{
  // Check bounds of tol. Tol is input as an angle measurement, but used internally as the sine of
  // that angle.
  if (tol > M_PI_2)
  {
    ROS_WARN("Global filter tolerance set above pi/2 (%lf), using %f", tol, M_PI_2);
    tol = 1.; // M_PI_2
  }
  else if (tol < 0.)
  {
    ROS_WARN("Global filter tolerance set below 0 (%lf), using 0", tol);
    tol = 0.;
  }
  else
  {
    tol = std::sin(tol);
  }

  // Starting with 1st segment, create edges that are all colinear within tol of each other.
  PolygonBoundary::iterator start = boundary.begin();
  if (boost::next(start) == boundary.end())
  {
    return;
  }
  PolygonSegment start_seg(*start, *boost::next(start));
  while (ros::ok())
  {
    if (boost::next(start, 2) == boundary.end())
    {
      return;
    }
    PolygonSegment next_seg(*boost::next(start), *boost::next(start, 2));
    double kross = start_seg.cross(next_seg);
    if (kross * kross / start_seg.length2() / next_seg.length2() < tol)
    { /* found linear segment */
      boundary.erase(boost::next(start));
      start_seg.end = *boost::next(start);
    }
    else
    { /* move to next segment */
      ++start;
      start_seg.start = *start;
      start_seg.end = *boost::next(start);
    }
  }
}

bool intersects(const PolygonBoundary& a, const PolygonBoundary& b)
{
  std::vector<PolygonSegment> v_a, v_b;
  boundaryToSegments(v_a, a);
  boundaryToSegments(v_b, b);
  return intersects(v_a, v_b);
}

bool intersects(const std::vector<PolygonSegment>& a, const std::vector<PolygonSegment>& b)
{
  BOOST_FOREACH (const PolygonSegment& seg_a, a)
  {
    BOOST_FOREACH (const PolygonSegment& seg_b, b)
    {
      if (seg_a.intersects(seg_b))
      {
        return true;
      }
    }
  }
  return false;
}

bool checkBoundary(const PolygonBoundary& bnd)
{
  const size_t N = bnd.size();
  if (N < 3)
  {
    return false;
  }

  // Check for 0-length segments
  const double LENGTH_TOL = 100. * std::numeric_limits<double>::epsilon();
  PolygonBoundary::const_iterator pt, last_pt;
  int pt_counter = 0;
  for (pt = bnd.begin(), last_pt = boost::prior(bnd.end()); pt != bnd.end(); ++pt)
  {
    if (pt == last_pt)
    {
      if (pt->dist(bnd.front()) < LENGTH_TOL)
      {

        ROS_WARN_STREAM("Invisible polygon boundary segment at point " << pt_counter);
        return false;
      }
    }
    else if (pt->dist(*boost::next(pt)) < LENGTH_TOL)
    {
      ROS_WARN_STREAM("Invisible polygon boundary segment at point " << pt_counter);
      return false;
    }
    pt_counter++;
  }

  // Subsequent checks for intersection are pointless on a triangle
  if (N == 3)
  {
    return true;
  }

  // Represent entire boundary as segments, and check for self-intersection
  std::vector<PolygonSegment> segments;
  boundaryToSegments(segments, bnd);

  for (std::vector<PolygonSegment>::const_iterator seg = segments.begin();
       seg != (segments.end() - 2); ++seg)
  {
    for (std::vector<PolygonSegment>::const_iterator other_seg = boost::next(seg, 2);
         other_seg != segments.end(); ++other_seg)
    {
      if (seg == segments.begin() && other_seg == boost::prior(segments.end()))
      {
        continue;
      }
      if (seg->intersects(*other_seg))
      {
        ROS_WARN_STREAM("Self-intersecting polygon at segments "
                        << seg - segments.begin() << " - " << other_seg - segments.begin() << " / "
                        << segments.size());
        return false;
      }
    }
  }

  return true;
}

bool checkBoundaryCollection(const PolygonBoundaryCollection& pbc)
{
  // Check each boundary individually
  BOOST_FOREACH (const PolygonBoundary& bnd, pbc)
  {
    if (!checkBoundary(bnd)) /*call check boundary without intersection checks*/
    {
      return false;
    }
  }

  // Precompute vectors of segments for polygonboundaries
  typedef std::vector<std::vector<PolygonSegment> > SegmentsList;
  SegmentsList segments_list;
  BOOST_FOREACH (const PolygonBoundary& bnd, pbc)
  {
    std::vector<PolygonSegment> segments;
    boundaryToSegments(segments, bnd);
    segments_list.push_back(segments);
  }

  //  // Check global intersections between boundaries
  //  BOOST_FOREACH(const PolygonBoundary &a, pbc)
  //  {
  //    BOOST_FOREACH(const PolygonBoundary &b, pbc)
  //    {
  //      if ((&a != &b) && intersects(a,b) )
  //      {
  //        return false;
  //      }
  //    }
  //  }

  // Check global intersections between boundaries
  for (SegmentsList::const_iterator a = segments_list.begin(); a != segments_list.end(); ++a)
  {
    for (SegmentsList::const_iterator b = boost::next(a); b != segments_list.end(); ++b)
    {
      if (intersects(*a, *b))
      {
        return false;
      }
    }
  }

  return true;
}

std::pair<size_t, float> closestPoint(const PolygonPt& pt, const PolygonBoundary& bnd)
{
  size_t point_num;
  double close_dist2(std::numeric_limits<double>::max());
  for (size_t idx = 0, max = bnd.size() - 1; idx != max; ++idx)
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
