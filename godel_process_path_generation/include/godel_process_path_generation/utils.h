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
 * utils.h
 *
 *  Created on: May 21, 2014
 *      Author: Dan Solomon
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <vector>
#include <cmath>
#include <Eigen/Geometry>
#include <algorithm>
#include <boost/graph/adjacency_list.hpp>
#include <boost/foreach.hpp>
#include <geometry_msgs/Polygon.h>
#include <visualization_msgs/MarkerArray.h>
#include "godel_process_path_generation/polygon_pts.hpp"

using std::cos;
using std::sin;
using std::asin;

namespace godel_process_path
{

namespace utils
{

namespace geometry
{

template <class Pt>
inline std::vector<Pt> discretizeArc2D(const Pt& p1, const Pt& p2, const Pt& c,
                                       bool positive_rotation, double max_sep)
{
  std::vector<Pt> vec;
  vec.push_back(p1); // Add first point

  // Find angle from p1 to p2. Check direction of theta against known direction of arc.
  double r = c.dist(p2); // TODO check for equal radius from c-p1?
  double theta = std::atan2((p1 - c).cross(p2 - c), (p1 - c).dot(p2 - c));
  if (theta < 0. && positive_rotation)
  {
    theta += 2. * M_PI;
  }
  else if (theta > 0. && !positive_rotation)
  {
    theta -= 2. * M_PI;
  }

  double max_theta(max_sep / r);
  if (std::abs(theta) > max_theta)
  {
    // Perform rotation by transform matrix: rotate p1 by theta about central axis
    size_t new_ptcnt = static_cast<size_t>(std::ceil(std::abs(theta / max_theta)) - 1.);
    double disc_angle = theta / static_cast<double>(new_ptcnt + 1.);
    Eigen::Vector3d start_pt(p1.x - c.x, p1.y - c.y,
                             1.); // Start point expressed in local frame (c in global frame)
    for (size_t ii = 1; ii <= new_ptcnt; ++ii)
    {
      double angle = disc_angle * static_cast<double>(ii); // what angle this point is added at
      Eigen::Matrix3d rotator;
      rotator << cos(angle), -sin(angle), c.x, sin(angle), cos(angle), c.y, 0, 0, 1;
      Eigen::Vector3d new_pt = rotator * start_pt; // New point in global frame
      vec.push_back(Pt(new_pt(0), new_pt(1)));
    }
  }
  return vec;
}

template <class Pt>
inline std::vector<Pt> discretizeLinear(const Pt& p1, const Pt& p2, double max_sep)
{
  // add [start-end) with interpolated points between
  std::vector<Pt> vec;
  vec.push_back(p1); // Add first point

  double sep = p1.dist(p2);
  if (sep > max_sep)
  {
    size_t new_ptcnt = static_cast<size_t>(std::ceil(sep / max_sep) - 1.);
    double disc_dist = sep / static_cast<double>(new_ptcnt + 1.);
    for (size_t ii = 1; ii <= new_ptcnt; ++ii)
    {
      vec.push_back(p1 + (p2 - p1) * (static_cast<double>(ii) * disc_dist / sep));
    }
  }
  return vec;
}

} /* namespace geometry */

namespace translations
{

/**@brief Convert a godel type to a geometry_msg type. This function operates on
 * PolygonBoundaryCollection and vector<Polygon>.
 * @param polygons_msg vector of Polygons populated from PolygonBoundaryCollection. Z-value is
 * unchanged.
 * @param pbc Collection of PolygonBoundaries.
 */
inline void godelToGeometryMsgs(std::vector<geometry_msgs::Polygon>& polygons_msg,
                                const godel_process_path::PolygonBoundaryCollection& pbc)
{
  polygons_msg.clear();
  BOOST_FOREACH (::godel_process_path::PolygonBoundary polygon, pbc)
  {
    geometry_msgs::Polygon polygon_msg;
    BOOST_FOREACH (godel_process_path::PolygonPt pt, polygon)
    {
      geometry_msgs::Point32 pt_msg;
      pt_msg.x = pt.x;
      pt_msg.y = pt.y;
      polygon_msg.points.push_back(pt_msg);
    }
    polygons_msg.push_back(polygon_msg);
  }
}

/**@brief Convert a geometry_msg type to a godel type. This function operates on vector<Polygon> and
 * PolygonBoundaryCollection.
 * @param pbc Collection of PolygonBoundaries.
 * @param polygons_msg vector of Polygons populated from PolygonBoundaryCollection. Z-value is
 * ignored.
 */
inline void geometryMsgsToGodel(godel_process_path::PolygonBoundaryCollection& pbc,
                                const std::vector<geometry_msgs::Polygon>& polygons_msg)
{
  pbc.clear();
  BOOST_FOREACH (geometry_msgs::Polygon polygon_msg, polygons_msg)
  {
    godel_process_path::PolygonBoundary polygon;
    BOOST_FOREACH (geometry_msgs::Point32 pt, polygon_msg.points)
    {
      polygon.push_back(godel_process_path::PolygonPt(pt.x, pt.y));
    }
    pbc.push_back(polygon);
  }
}

/**@brief Convert a godel type to a visualization_msg type. This function operates on
 * PolygonBoundary and Marker.
 * Creates a line list with default color and 1mm thickness.
 * User's responsibility to complete remainder of message.
 * @param marker Marker msg populated with points
 * @param polygon PolygonBoundary containing pts of polygon.
 * @param default_color Color to use for all points in marker.
 * @param default_scale Line width.
 */
inline void godelToVisualizationMsgs(visualization_msgs::Marker& marker,
                                     const godel_process_path::PolygonBoundary& polygon,
                                     std_msgs::ColorRGBA default_color = std_msgs::ColorRGBA(),
                                     double default_scale = .001)
{
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  BOOST_FOREACH (::godel_process_path::PolygonPt pt, polygon)
  {
    geometry_msgs::Point pt_msg;
    pt_msg.x = pt.x;
    pt_msg.y = pt.y;
    marker.points.push_back(pt_msg);
  }
  marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), default_color);
  marker.scale.x = default_scale;
}

/**@brief Convert a godel type to a visualization_msg type. This function operates on
 * PolygonBoundaryCollection and MarkerArray.
 * Creates a series of line lists with default color and 1mm thickness.
 * User's responsibility to complete remainder of message.
 * @param marker MarkerArray msg populated with line list markers. One marker per PolygonBoundary
 * @param pbc Collection of PolygonBoundaries.
 * @param default_color Color to use for all points in all markers.
 * @param default_scale Line width for all markers.
 */
inline void godelToVisualizationMsgs(visualization_msgs::MarkerArray& markers,
                                     const godel_process_path::PolygonBoundaryCollection& pbc,
                                     std_msgs::ColorRGBA default_color = std_msgs::ColorRGBA(),
                                     double default_scale = .001)
{
  markers.markers.clear();
  BOOST_FOREACH (::godel_process_path::PolygonBoundary polygon, pbc)
  {
    visualization_msgs::Marker marker;
    godelToVisualizationMsgs(marker, polygon, default_color, default_scale);
    markers.markers.push_back(marker);
  }
}

} /* namespace translations */

/* General utility functions */

/**@brief Wrapper for std::find()
 * @return True if item exists in container.
 */
template <class Item, class Container>
inline bool exists(const Item& item, const Container& container)
{
  return std::find(container.begin(), container.end(), item) != container.end();
}

/**@brief Find first child of a vertex in a boost graph
 * @param[out] child First child vertex of parent (if it exists)
 * @param[in] parent Vertex to find child of
 * @param[in] g Graph to search through
 * @return True if child found. If return is false, child left unchanged.
 */
template <class Graph>
inline bool getFirstChild(typename ::boost::graph_traits<Graph>::vertex_descriptor& child,
                          const typename ::boost::graph_traits<Graph>::vertex_descriptor& parent,
                          const Graph& g)
{
  typename boost::graph_traits<Graph>::out_edge_iterator edge, edges_end;
  boost::tie(edge, edges_end) = boost::out_edges(parent, g);
  if (edge == edges_end)
  {
    return false;
  }
  child = boost::target(*edge, g);
  return true;
}

/**@brief Move an item from from container to another.
 * Designed to work with STL iterators
 * @param from Container containing iterator. Iter is removed from  container
 * @param to Container to move to. If successfully completed, item referenced by "iter" is added to
 * "to".
 * @param iter Iterator into "from" container
 * @return True if item moved successfully, false if "from" does not contain "iter"
 */
template <class Container>
inline bool moveIterFrom(Container& from, Container& to, typename Container::iterator iter)
{
  if (iter >= from.begin() && iter < from.end())
  {
    to.push_back(*iter);
    from.erase(iter);
    return true;
  }
  return false;
}

/**@brief Move an item from from container to another.
 * Designed to work with STL iterators
 * @param from Container containing "item". Item is removed from container.
 * @param to Container to move to. If successfully completed, item added to "to".
 * @param item Item in "from" container
 * @return True if item moved successfully, false if "from" does not contain "item"
 */
template <class Container, class Item>
inline bool moveItemFrom(Container& from, Container& to, const Item& item)
{
  typename Container::iterator iter = std::find(from.begin(), from.end(), item);
  if (iter != from.end())
  {
    moveIterFrom(from, to, iter);
    return true;
  }
  return false;
}

} /* end utils */
} /* end godel_process_path */
#endif /* UTILS_H_ */
