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
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "godel_process_path_generation/process_path_generator.h"
#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/polygon_utils.h"
#include "godel_process_path_generation/utils.h"

using descartes::ProcessPt;

namespace godel_process_path
{

void operator<<(ProcessPt& pr_pt, const PolygonPt& pg_pt)
{
  pr_pt.setPosePosition(pg_pt.x, pg_pt.y, 0.);
}

void ProcessPathGenerator::addInterpolatedProcessPts(const ProcessPt& start, const ProcessPt& end)
{
  const Eigen::Affine3d& p1 = start.pose();
  const Eigen::Affine3d& p2 = end.pose();
  double sep = (p2.translation() - p1.translation()).norm();
  if (sep > max_discretization_distance_)
  {
    size_t new_ptcnt = static_cast<size_t>(std::ceil(sep / max_discretization_distance_) - 1.);
    for (size_t ii = 1; ii <= new_ptcnt; ++ii)
    {
      double t = static_cast<double>(ii) / static_cast<double>(new_ptcnt + 1.);
      Eigen::Vector3d pos = (1 - t) * p1.translation() + t * p2.translation();
      Eigen::Quaterniond rot(
          Eigen::Quaterniond(p1.rotation()).slerp(t, Eigen::Quaterniond(p2.rotation())));

      ProcessPt new_pt = start;
      new_pt.pose() = Eigen::Translation3d(pos) * rot;
      process_path_.addPoint(new_pt);
    }
  }
}

void ProcessPathGenerator::addPolygonToProcessPath(const PolygonBoundary& bnd_ref)
{
  PolygonBoundary bnd = bnd_ref;
  bnd.push_back(bnd.front());
  ProcessPt process_pt;
  BOOST_FOREACH (const PolygonPt& pg_pt, bnd)
  {
    process_pt << pg_pt;
    process_path_.addPoint(process_pt);
  }
}

void ProcessPathGenerator::addTraverseToProcessPath(const PolygonPt& from, const PolygonPt& to)
{
  ProcessPt start, /*last point from previous path*/
      retract,     /*safe point above previous path*/
      approach,    /*safe point above next path*/
      end;         /*start point of next path*/

  start << from;
  retract.setPosePosition(from.x, from.y, safe_traverse_height_);
  approach.setPosePosition(to.x, to.y, safe_traverse_height_);
  end << to;

  addInterpolatedProcessPts(start, retract);
  process_path_.addPoint(retract); /*addInterpolatedPoints does not add endpoints */
  addInterpolatedProcessPts(retract, approach);
  process_path_.addPoint(approach);
  addInterpolatedProcessPts(approach, end); // TODO Moving to 1st pt of next
                                                                // path is at vel.blending instead
                                                                // of vel.approach
}

bool ProcessPathGenerator::createProcessPath()
{
  if (!variables_ok())
  {
    ROS_WARN_STREAM("Problem with variable values in ProcessPathGenerator, were they set properly?"
                    << std::endl
                    << "Tool Radius: " << tool_radius_ << "(m)" << std::endl
                    << "Margin: " << margin_ << "(m)" << std::endl
                    << "Overlap: " << overlap_ << "(m)");
    return false;
  }

  /* Create a series of polygons to represent the paths for blending
   * Polygons are ordered so as to begin at most inner, spiral out to most outer,
   *  jump to next incomplete inner, spiral out to largest incomplete outer, etc. */
  if (!path_polygons_ || !path_offsets_)
  {
    ROS_WARN("Must set path polygons and path offsets before creating process path.");
    return false;
  }

  if (path_polygons_->size() != path_offsets_->size())
  {
    ROS_WARN("Must have identical count of Polygons and Offsets.");
    return false;
  }

  /* Strategy: Create initial approach
   * Do loops until offset increases; addTraverseToProcessPath
   * Create retract */

  // Add approach vector
  process_path_.clear();
  ProcessPt approach, start;
  const PolygonPt& first = *(path_polygons_->begin()->begin());

  approach.setPosePosition(first.x, first.y, safe_traverse_height_);
  start << first;
  process_path_.addPoint(approach);
  addInterpolatedProcessPts(approach, start);
  ROS_INFO_COND(verbose_, "Created approach path.");

  // Do all loops until last
  double current_offset = std::numeric_limits<double>::max();
  size_t pgIdx(0);
  while (pgIdx < path_polygons_->size() - 1)
  {
    current_offset = path_offsets_->at(pgIdx);
    const PolygonBoundary& polygon = path_polygons_->at(pgIdx);
    addPolygonToProcessPath(polygon);
    ROS_INFO_COND(verbose_, "Added polygon %li to process path.", pgIdx);

    const PolygonPt last_pgpt = polygon.front(); // Each polygon ends where it starts
    ProcessPt last_pt;
    last_pt << last_pgpt;

    ++pgIdx;
    PolygonBoundary& next_polygon = path_polygons_->at(pgIdx);

    if (path_offsets_->at(pgIdx) < current_offset)
    { /*Take one step out*/
      size_t rotate_index = polygon_utils::closestPoint(last_pgpt, next_polygon).first;
      std::rotate(next_polygon.begin(), next_polygon.begin() + rotate_index, next_polygon.end());
      ProcessPt next_pt;
      next_pt << next_polygon.front();
      addInterpolatedProcessPts(last_pt, next_pt);
      ROS_INFO_COND(verbose_, "Added connection to polygon %li.", pgIdx);
    }
    else
    {
      addTraverseToProcessPath(last_pgpt, next_polygon.front());
      ROS_INFO_COND(verbose_, "Added traverse to polygon %li.", pgIdx);
    }
  }

  // Add last loop and retract
  const PolygonBoundary& polygon = path_polygons_->at(pgIdx);
  addPolygonToProcessPath(polygon);
  ROS_INFO_COND(verbose_, "Added polygon %li to process path.", pgIdx);
  const PolygonPt last_pgpt = polygon.front();
  ProcessPt last, retract;
  last << last_pgpt;
  retract.setPosePosition(last_pgpt.x, last_pgpt.y, safe_traverse_height_);
  addInterpolatedProcessPts(last, retract);
  process_path_.addPoint(retract);
  ROS_INFO_COND(verbose_, "Added retract path.");

  ROS_INFO_COND(verbose_, "Successfully converted Polygons to ProcessPath.");
  return true;
}

} /* namespace godel_process_path */
