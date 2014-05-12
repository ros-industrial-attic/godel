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
 * process_path_generator.cpp
 *
 *  Created on: May 9, 2014
 *      Author: ros
 */

#include <ros/ros.h>
#include "godel_process_path_generation/process_path_generator.h"
#include <openvoronoi/offset.hpp>
#include <openvoronoi/polygon_interior_filter.hpp>
#include <boost/tuple/tuple.hpp>


namespace ovd
{
  typedef boost::graph_traits<MachiningGraph>::out_edge_iterator MGOutEdgeIt;
}

namespace godel_process_path
{

typedef std::vector<ovd::MGVertex> MachiningLoopList;
bool getChild(const ovd::MGVertex &parent, const ovd::MachiningGraph &mg, ovd::MGVertex &child);
void moveLoopItem(const ovd::MGVertex &item, MachiningLoopList &from, MachiningLoopList &to);
bool exists(const ovd::MGVertex &item, const MachiningLoopList &container);


bool ProcessPathGenerator::configure(PolygonBoundaryCollection boundaries)
{
  for (PolygonBoundaryCollection::const_iterator boundary=boundaries.begin(), bs_end=boundaries.end(); boundary!=bs_end; ++boundary)
  {
    std::vector<int> pt_id;
    pt_id.reserve(boundary->size());
    bool first=true;
    for (PolygonBoundary::const_iterator pt=boundary->begin(), b_end=boundary->end(); pt!=b_end; ++pt)
    {
      int id = vd_->insert_point_site(ovd::Point(pt->x, pt->y));
      if (first)
      {
        first=false;
      }
      else
      {
        vd_->insert_line_site(pt_id.back(), id);
      }
      pt_id.push_back(id);
    }
    vd_->insert_line_site(pt_id.back(), pt_id.front());

  }
  if (!vd_->check())
  {
    ROS_ERROR("Voroni diagram check failed.");
    configure_ok_ = false;
  }
  else
  {
    ovd::polygon_interior_filter pi(true);
    vd_->filter(&pi);

    configure_ok_ = true;
  }

  return configure_ok_;
}

bool ProcessPathGenerator::createOffsetPolygons(PolygonBoundaryCollection &polygons, std::vector<double> &offset_depths)
{
  ovd::HEGraph& g = vd_->get_graph_reference();
  ovd::Offset offsetter(g);
  ovd::OffsetSorter sorter(g);

  // Perform offsets
  double offset_distance(tool_radius_ + margin_);
  size_t loop_count(0);
  while (true)
  {
    ovd::OffsetLoops offset_list = offsetter.offset(offset_distance);
    if (offset_list.size() == 0)
    {
      break;
    }
    loop_count += offset_list.size();
    for (ovd::OffsetLoops::const_iterator loop=offset_list.begin(), loops_end=offset_list.end(); loop!=loops_end; ++loop)
    {
      sorter.add_loop(*loop);
    }
    offset_distance += (2. * tool_radius_ - overlap_);
  }
  if (loop_count == 0)
  {
    ROS_WARN_STREAM("No offsets were generated: " << std::endl <<
                    "Tool Radius: " << tool_radius_ << " (m)" << std::endl <<
                    "Margin: " << margin_ << " (m)" << std::endl <<
                    "Initial offset: " << offset_distance << " (m)");
    return false;
  }

  sorter.sort_loops();
  const ovd::MachiningGraph &mg = sorter.getMachiningGraph();

  MachiningLoopList ordered_loops, unordered_loops;       // For tracking order of loops for machining
//  typedef std::vector<ovd::MGVertex>::iterator MGLoopListItr;

  // Populate unordered loops with all graph vertices
  ovd::MGVertexItr loop, vertex_end;
  for (boost::tie(loop, vertex_end) = boost::vertices(mg); loop!=vertex_end; ++loop)
  {
    unordered_loops.push_back(*loop);
  }

  /* Sort unordered loops into ordered loops:
   *  Move deepest loop in unordered to ordered
   *  Move each parent from unordered to ordered until no more parents
   *  Repeat
   */
  while (unordered_loops.size() != 0)
  {
    // Find deepest loop
    double largest_offset(0);
    ovd::MGVertex deepest_loop;
    BOOST_FOREACH(ovd::MGVertex loop_descriptor, unordered_loops)
    {
      if (mg[loop_descriptor].offset_distance > largest_offset)
      {
        largest_offset = mg[loop_descriptor].offset_distance;
        deepest_loop = loop_descriptor;
      }
    }
    moveLoopItem(deepest_loop, unordered_loops, ordered_loops);

    // Find child of deepest loop (TODO check for multiple children, should never happen!)
    // Move child to ordered
    ovd::MGVertex child;
    while (getChild(ordered_loops.back(), mg, child))
    {
      if (exists(child, unordered_loops))
      {
        moveLoopItem(child, unordered_loops, ordered_loops);
      }
      else
      {
        break;
      }
    }
  }

//  for (LoopItr loop_it=ordered_loops.begin(); loop_it != ordered_loops.end(); ++loop_it)
  BOOST_FOREACH(ovd::MGVertex loop_descriptor, ordered_loops)
  {
    PolygonBoundary path;
    const ovd::OffsetLoop &loop = mg[loop_descriptor];
    const ovd::OffsetVertex &prior_vtx = loop.vertices.front();
    for (std::list<ovd::OffsetVertex>::const_iterator vtx=boost::next(loop.vertices.begin()); vtx!=loop.vertices.end(); ++vtx)
    {
        discretizeSegment(prior_vtx, *vtx, path);
    }
    polygons.push_back(path);
    offset_depths.push_back(loop.offset_distance);
  }

  return true;
}

bool ProcessPathGenerator::createProcessPath()
{
  if (!variables_ok())
  {
    ROS_WARN_STREAM("Problem with variable values in ProcessPathGenerator, were they set properly?" << std::endl <<
                    "Tool Radius: " << tool_radius_ << "(m)" << std::endl <<
                    "Margin: " << margin_ << "(m)" << std::endl <<
                    "Overlap: " << overlap_ << "(m)");
    return false;
  }
  if (!configure_ok_)
  {
    ROS_WARN("Configuration incomplete. Run configure() successfully before creating process path.");
    return false;
  }

  /* Create a series of polygons to represent the paths for blending
   * Polygons are ordered so as to begin at most inner, spiral out to most outer,
   *  jump to next incomplete inner, spiral out to largest incomplete outer, etc. */
  PolygonBoundaryCollection polygons;
  std::vector<double> offset_depths;    // Each depth corresponds to item in polygons
  if (!createOffsetPolygons(polygons, offset_depths))
  {
    ROS_WARN("Failure during offset procedure.");
    return false;
  }



//  addPolygonToProcessPath(bnd);



  return true;
}

void ProcessPathGenerator::discretizeSegment(const ovd::OffsetVertex &p1, const ovd::OffsetVertex &p2, PolygonBoundary &bnd) const
{
  if (p1.r == -1 or p2.r == -1)
  {
    discretizeLinear(p1, p2, bnd);
  }
  else
  {
    discretizeArc(p1, p2, bnd);
  }
}


bool exists(const ovd::MGVertex &item, const MachiningLoopList &container)
{
  for (MachiningLoopList::const_iterator iter=container.begin(); iter !=container.end(); ++iter)
  {
    if (item == *iter)
    {
      return true;
    }
  }
  return false;
}

bool getChild(const ovd::MGVertex &parent, const ovd::MachiningGraph &mg, ovd::MGVertex &child)
{
  ovd::MGOutEdgeIt edge, edge_end;
  boost::tie(edge, edge_end) = boost::out_edges(parent, mg);
  if (edge == edge_end)
  {
    return false;
  }
  child = boost::target(*edge, mg);
  return true;
}

void moveLoopItem(const ovd::MGVertex &item, MachiningLoopList &from, MachiningLoopList &to)
{
  // delete item from, add item to
  MachiningLoopList::iterator iter;
  for (iter=from.begin(); iter !=from.end(); ++iter)
  {
    if (item == *iter)
    {
      from.erase(iter);
      break;
    }
  }
  ROS_ASSERT(iter != from.end());
  to.push_back(item);
}


} /* namespace godel_process_path */
