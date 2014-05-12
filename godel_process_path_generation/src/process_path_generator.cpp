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
#include <openvoronoi/offset_sorter.hpp>
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
    return false;
  }

  ovd::polygon_interior_filter pi(true);
  vd_->filter(&pi);

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

  // Perform initial offset
  ovd::HEGraph& g = vd_->get_graph_reference();
  ovd::Offset offsetter(g);
  ovd::OffsetSorter sorter(g);
  double offset_distance(tool_radius_ + margin_);

  ovd::OffsetLoops offset_list = offsetter.offset(offset_distance);
  if (offset_list.size() == 0)
  {
    ROS_WARN_STREAM("No path was generated for initial offset: " << offset_distance << std::endl <<
                    "Tool Radius: " << tool_radius_ << "(m)" << std::endl <<
                    "Margin: " << margin_ << "(m)");
    return false;
  }
  for (ovd::OffsetLoops::const_iterator loop=offset_list.begin(), loops_end=offset_list.end(); loop!=loops_end; ++loop)
  {
    sorter.add_loop(*loop);
  }

  // Perform subsequent offsets
  while (true)
  {
    offset_distance += (2. * tool_radius_ - overlap_);
    offset_list = offsetter.offset(offset_distance);
    if (offset_list.size() == 0)
    {
      break;
    }
    for (ovd::OffsetLoops::const_iterator loop=offset_list.begin(), loops_end=offset_list.end(); loop!=loops_end; ++loop)
    {
      sorter.add_loop(*loop);
    }
  }

  sorter.sort_loops();
  ovd::MachiningGraph mg = sorter.getMachiningGraph();
  size_t num_vertices = boost::num_vertices(mg);
  MachiningLoopList ordered_loops(num_vertices), unordered_loops(num_vertices);       // For tracking order of loops for machining
  typedef std::vector<ovd::MGVertex>::iterator LoopItr;

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
    LoopItr deepest_loop;
    for (LoopItr loop = unordered_loops.begin(); loop!=unordered_loops.end(); ++loop)
    {
      if (mg[*loop].offset_distance > largest_offset)
      {
        largest_offset = mg[*loop].offset_distance;
        deepest_loop = loop;
      }
    }

    // Move deepest loop to ordered
//    moveLoopItem(*loop, unordered_loops, ordered_loops);
    ordered_loops.push_back(*deepest_loop);
    unordered_loops.erase(deepest_loop);

    // Find child of deepest loop (TODO check for multiple children, should never happen!)
    // Move child to ordered
    ovd::MGVertex child;
    while (getChild(ordered_loops.back(), mg, child))
    {
      moveLoopItem(child, unordered_loops, ordered_loops);
    }
  }



  return true;
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
