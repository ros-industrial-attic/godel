/*
 * Software License Agreement (GPLv3 License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * This file is part of godel. https://github.com/ros-industrial-consortium/godel
 *
 *  godel_polygon_offset is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  godel_polygon_offset is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with godel_polygon_offset.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * polygon_offset.cpp
 *
 *  Created on: May 24, 2014
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <openvoronoi/offset.hpp>
#include <openvoronoi/polygon_interior_filter.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>
#include <boost/next_prior.hpp>
#include <godel_process_path_generation/utils.h>
#include "godel_polygon_offset/polygon_offset.h"

using godel_process_path::PolygonBoundaryCollection;
using godel_process_path::PolygonBoundary;
using godel_process_path::PolygonPt;
using godel_process_path::utils::exists;
using godel_process_path::utils::getFirstChild;
using godel_process_path::utils::moveItemFrom;

typedef std::vector<ovd::MGVertex> MachiningLoopList;

namespace godel_polygon_offset
{

bool PolygonOffset::init(const PolygonBoundaryCollection& pbc, double _offset,
                         double _initial_offset, double _discretization)
{
  if (_offset <= 0. || _initial_offset <= 0. || _discretization <= 0)
  {
    ROS_ERROR("Cannot initialize PolygonOffset with negative offset parameters.");
    return false;
  }

  // Reset PolygonOffset for new operations.
  vd_.reset(new ovd::VoronoiDiagram(1, 100));
  offset_ = _offset;
  initial_offset_ = _initial_offset;
  discretization_ = _discretization;
  ROS_INFO_COND(verbose_, "Creating voroni diagram from polygons");

  // Add all points to vd.
  std::vector<std::vector<int> > pt_id_collection;
  PolygonBoundaryCollection::const_iterator boundary, bs_end;
  for (boundary = pbc.begin(), bs_end = pbc.end(); boundary != bs_end; ++boundary)
  {
    std::vector<int> pt_id;
    bool first = true;
    for (PolygonBoundary::const_iterator pt = boundary->begin(), b_end = boundary->end();
         pt != b_end; ++pt)
    {
      pt_id.push_back(vd_->insert_point_site(ovd::Point(pt->x, pt->y)));
      ROS_INFO_COND(verbose_, "Added point %i at location %f, %f", pt_id.back(), pt->x, pt->y);
    }
    pt_id_collection.push_back(pt_id);
  }

  // Connect points into boundaries.
  for (boundary = pbc.begin(); boundary != bs_end; ++boundary)
  {
    std::vector<int>& pt_id = pt_id_collection.at(boundary - pbc.begin());
    for (size_t ii = 0; ii < pt_id.size() - 1; ++ii)
    {
      ROS_INFO_COND(verbose_, "Adding line from pt %i to pt %i", pt_id.at(ii), pt_id.at(ii + 1));
      vd_->insert_line_site(pt_id.at(ii), pt_id.at(ii + 1));
    }
    ROS_INFO_COND(verbose_, "Closing loop from pt %i to pt %i", pt_id.back(), pt_id.front());
    vd_->insert_line_site(pt_id.back(), pt_id.front());
  }

  // Check vd for validity, filter interior points.
  if (!vd_->check())
  {
    ROS_ERROR("Voroni diagram check failed.");
    init_ok_ = false;
  }
  else
  {
    ovd::polygon_interior_filter pi(true);
    vd_->filter(&pi);

    init_ok_ = true;
  }

  ROS_INFO_COND(verbose_ && init_ok_, "Configure complete.");
  return init_ok_;
}

bool PolygonOffset::generateOrderedOffsets(PolygonBoundaryCollection& polygons,
                                           std::vector<double>& offsets)
{
  if (!init_ok_)
  {
    ROS_ERROR("Cannot use PolygonOffset without calling init() first.");
    return false;
  }

  ovd::HEGraph& g = vd_->get_graph_reference();
  ovd::Offset offsetter(g);
  ovd::OffsetSorter sorter(g);

  /* Perform offsets:
   * Start with initial_offset, and proceed with offset distance until no further offsets are
   * generated.
   * Add each offset to sorter for subsequent sorting. */
  double offset_distance(initial_offset_);
  size_t loop_count(0);
  while (true)
  {
    ROS_INFO_COND(verbose_, "Creating offset with distance %f", offset_distance);
    ovd::OffsetLoops offset_list = offsetter.offset(offset_distance);
    if (offset_list.size() == 0)
    {
      break;
    }
    loop_count += offset_list.size();
    for (ovd::OffsetLoops::const_iterator loop = offset_list.begin(), loops_end = offset_list.end();
         loop != loops_end; ++loop)
    {
      sorter.add_loop(*loop);
    }
    offset_distance += offset_;
  }
  if (loop_count == 0)
  {
    ROS_WARN_STREAM("No offsets were generated: " << std::endl
                                                  << "Initial Offset: " << initial_offset_
                                                  << " (m)");
    return false;
  }
  ROS_INFO_COND(verbose_, "Created %li offset loops", loop_count);

  // Perform initial sort.
  sorter.sort_loops();
  const ovd::MachiningGraph& mg = sorter.getMachiningGraph();

  MachiningLoopList ordered_loops, unordered_loops; // For tracking order of loops for machining

  // Populate unordered loops with all graph vertices
  ovd::MGVertexItr loop, vertex_end;
  for (boost::tie(loop, vertex_end) = boost::vertices(mg); loop != vertex_end; ++loop)
  {
    unordered_loops.push_back(*loop);
  }

  /* Sort unordered loops into ordered loops:
   *  Move deepest loop in unordered to ordered
   *  Move each parent from unordered to ordered until no more parents
   *  Repeat (find deepest loop...)
   */
  while (unordered_loops.size() != 0)
  {
    // Find deepest loop
    double largest_offset(0);
    ovd::MGVertex deepest_loop;
    BOOST_FOREACH (ovd::MGVertex loop_descriptor, unordered_loops)
    {
      if (mg[loop_descriptor].offset_distance > largest_offset)
      {
        largest_offset = mg[loop_descriptor].offset_distance;
        deepest_loop = loop_descriptor;
      }
    }
    ROS_INFO_COND(verbose_, "Moving loop at depth %f to ordered list.", largest_offset);
    moveItemFrom(unordered_loops, ordered_loops, deepest_loop);

    // Find child of deepest loop (TODO check for multiple children, should never happen!)
    // Move child to ordered
    ovd::MGVertex child;
    while (getFirstChild(child, ordered_loops.back(), mg))
    {
      if (exists(child, unordered_loops))
      {
        ROS_INFO_COND(verbose_, "Moving loop at depth %f to ordered list.",
                      mg[child].offset_distance);
        moveItemFrom(unordered_loops, ordered_loops, child);
      }
      else
      {
        break;
      }
    }
  }

  // Convert ovg::OffsetLoops to polygons and discretize
  // Populate polygons and offsets
  polygons.clear();
  offsets.clear();
  BOOST_FOREACH (ovd::MGVertex loop_descriptor, ordered_loops)
  {
    PolygonBoundary polygon;
    const ovd::OffsetLoop& loop = mg[loop_descriptor];
    ovd::OffsetVertex prior_vtx = loop.vertices.front();
    std::list<ovd::OffsetVertex>::const_iterator vtx = boost::next(loop.vertices.begin());
    while (vtx != loop.vertices.end())
    {
      PolygonPt prior(prior_vtx.p.x, prior_vtx.p.y), current(vtx->p.x, vtx->p.y);
      std::vector<PolygonPt> pts;
      if (vtx->r == -1)
      {
        pts =
            godel_process_path::utils::geometry::discretizeLinear(prior, current, discretization_);
      }
      else
      {
        PolygonPt arc_center(vtx->c.x, vtx->c.y);
        pts = godel_process_path::utils::geometry::discretizeArc2D(prior, current, arc_center,
                                                                   !vtx->cw, discretization_);
      }
      polygon.insert(polygon.end(), pts.begin(), pts.end());
      pts.clear();
      prior_vtx = *vtx;
      ++vtx;
    }
    polygons.push_back(polygon);
    offsets.push_back(loop.offset_distance);
  }

  return true;
}

} /* namespace godel_polygon_offset */
