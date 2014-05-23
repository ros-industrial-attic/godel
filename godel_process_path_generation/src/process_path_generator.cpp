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
#include "godel_process_path_generation/process_path_generator.h"
#include "godel_process_path_generation/polygon_pts.hpp"
#include <openvoronoi/offset.hpp>
#include <openvoronoi/polygon_interior_filter.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/foreach.hpp>


using descartes::ProcessPt;
using std::cos;
using std::sin;
using std::asin;

namespace godel_process_path
{

typedef std::vector<ovd::MGVertex> MachiningLoopList;

// Function signatures for code below
bool getChild(const ovd::MGVertex &parent, const ovd::MachiningGraph &mg, ovd::MGVertex &child);
void moveLoopItem(const ovd::MGVertex &item, MachiningLoopList &from, MachiningLoopList &to);
bool exists(const ovd::MGVertex &item, const MachiningLoopList &container);
void operator<<(ProcessPt &pr_pt, const PolygonPt &pg_pt);

void ProcessPathGenerator::addInterpolatedProcessPts(const ProcessPt &start, const ProcessPt &end, double vel)
{
  const Eigen::Affine3d &p1 = start.pose();
  const Eigen::Affine3d &p2 = end.pose();
  double sep = (p2.translation() - p1.translation()).norm();
  if (sep > max_discretization_distance_)
  {
    size_t new_ptcnt = static_cast<size_t>(std::ceil(sep/max_discretization_distance_) - 1.);
    for (size_t ii=1; ii<=new_ptcnt; ++ii)
    {
      double t = static_cast<double>(ii)/static_cast<double>(new_ptcnt+1.);
      Eigen::Vector3d pos = (1-t) * p1.translation() + t * p2.translation();
      Eigen::Quaterniond rot(Eigen::Quaterniond(p1.rotation()).slerp(t, Eigen::Quaterniond(p2.rotation())));

      ProcessPt new_pt = start;
      new_pt.pose() = Eigen::Translation3d(pos) * rot;
      process_path_.addPoint(new_pt);
      process_path_.addTransition(velToTransition(vel));
    }
  }
}

void ProcessPathGenerator::addPolygonToProcessPath(const PolygonBoundary &bnd_ref, double vel)
{
  PolygonBoundary bnd = bnd_ref;
  bnd.push_back(bnd.front());
  ProcessPt process_pt;
  BOOST_FOREACH(const PolygonPt &pg_pt, bnd)
  {
    process_pt << pg_pt;
    process_path_.addPoint(process_pt);
    process_path_.addTransition(velToTransition(vel));
  }

}

void ProcessPathGenerator::addTraverseToProcessPath(const PolygonPt &from, const PolygonPt &to)
{
  ProcessPt start,      /*last point from previous path*/
            retract,    /*safe point above previous path*/
            approach,   /*safe point above next path*/
            end;        /*start point of next path*/

  start << from;
  retract.setPosePosition(from.x, from.y, safe_traverse_height_);
  approach.setPosePosition(to.x, to.y, safe_traverse_height_);
  end << to;

  addInterpolatedProcessPts(start, retract, velocity_.retract);
  process_path_.addPoint(retract);      /*addInterpolatedPoints does not add endpoints */
  process_path_.addTransition(velToTransition(velocity_.retract));
  addInterpolatedProcessPts(retract, approach, velocity_.traverse);
  process_path_.addPoint(approach);
  process_path_.addTransition(velToTransition(velocity_.traverse));
  addInterpolatedProcessPts(approach, end, velocity_.approach); //TODO Moving to 1st pt of next path is at vel.blending instead of vel.approach
}

bool ProcessPathGenerator::configure(PolygonBoundaryCollection boundaries)
{
  vd_.reset(new ovd::VoronoiDiagram(1,100));
  ROS_INFO_COND(verbose_, "Creating voroni diagram from polygons");

  std::vector<std::vector<int> > pt_id_collection;
  PolygonBoundaryCollection::const_iterator boundary, bs_end;
  for (boundary=boundaries.begin(), bs_end=boundaries.end(); boundary!=bs_end; ++boundary)
  {
    std::vector<int> pt_id;
    bool first=true;
    for (PolygonBoundary::const_iterator pt=boundary->begin(), b_end=boundary->end(); pt!=b_end; ++pt)
    {
      pt_id.push_back(vd_->insert_point_site(ovd::Point(pt->x, pt->y)));
      ROS_INFO_COND(verbose_, "Added point %i at location %f, %f", pt_id.back(), pt->x, pt->y);
    }
    pt_id_collection.push_back(pt_id);
  }
  for (boundary=boundaries.begin(), bs_end=boundaries.end(); boundary!=bs_end; ++boundary)
  {
    std::vector<int> &pt_id = pt_id_collection.at(boundary-boundaries.begin());
    for (size_t ii=0; ii<pt_id.size()-1; ++ii)
    {
      ROS_INFO_COND(verbose_, "Adding line from pt %i to pt %i", pt_id.at(ii), pt_id.at(ii+1));
      vd_->insert_line_site(pt_id.at(ii), pt_id.at(ii+1));
    }
    ROS_INFO_COND(verbose_, "Closing loop from pt %i to pt %i", pt_id.back(), pt_id.front());
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

  ROS_INFO_COND(verbose_, "Configure complete.");
  return configure_ok_;
}

void ProcessPathGenerator::convertPolygonsToProcessPath(PolygonBoundaryCollection &polygons, const std::vector<double> &offsets)
{
  /* Strategy: Create initial approach
   * Do loops until offset increases; addTraverseToProcessPath
   * Create retract */

  // Add approach vector
  process_path_.clear();
  ProcessPt approach, start;
  const PolygonPt &first = *(polygons.begin()->begin());
  approach.setPosePosition(first.x, first.y, safe_traverse_height_);
  start << first;
  process_path_.addPoint(approach);
  addInterpolatedProcessPts(approach, start, velocity_.approach);

  // Do all loops until last
  double current_offset = std::numeric_limits<double>::max();
  size_t pgIdx(0);
  while (pgIdx < polygons.size()-1)
  {
    current_offset = offsets.at(pgIdx);
    PolygonBoundary &polygon = polygons.at(pgIdx);
    addPolygonToProcessPath(polygon, velocity_.blending);

    const PolygonPt last_pgpt = polygon.front(); //Each polygon ends where it starts
    ProcessPt last_pt;
    last_pt << last_pgpt;

    ++pgIdx;
    PolygonBoundary &next_polygon = polygons.at(pgIdx);

    if (offsets.at(pgIdx) < current_offset)
    {   /*Take one step out*/
      size_t rotate_index = closestPoint(last_pgpt, next_polygon).first;
      std::rotate(next_polygon.begin(), next_polygon.begin()+rotate_index, next_polygon.end());
      ProcessPt next_pt;
      next_pt << next_polygon.front();
      addInterpolatedProcessPts(last_pt, next_pt, velocity_.approach);
    }
    else
    {
      addTraverseToProcessPath(last_pgpt, next_polygon.front());
    }
  }

  // Add last loop and retract
  PolygonBoundary &polygon = polygons.at(pgIdx);
  addPolygonToProcessPath(polygon, velocity_.blending);
  const PolygonPt last_pgpt = polygon.front();
  ProcessPt last, retract;
  last << last_pgpt;
  retract.setPosePosition(last_pgpt.x, last_pgpt.y, safe_traverse_height_);
  addInterpolatedProcessPts(last, retract);
  process_path_.addPoint(retract);
  process_path_.addTransition(velToTransition(velocity_.retract));

  ROS_INFO_COND(verbose_, "Successfully converted Polygons to ProcessPath.");
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
    ROS_INFO_COND(verbose_, "Creating offset with distance %f", offset_distance);
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
  ROS_INFO_COND(verbose_, "Created %li offset loops", loop_count);

  sorter.sort_loops();
  const ovd::MachiningGraph &mg = sorter.getMachiningGraph();

  MachiningLoopList ordered_loops, unordered_loops;       // For tracking order of loops for machining

  // Populate unordered loops with all graph vertices
  ovd::MGVertexItr loop, vertex_end;
  for (boost::tie(loop, vertex_end) = boost::vertices(mg); loop!=vertex_end; ++loop)
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
    BOOST_FOREACH(ovd::MGVertex loop_descriptor, unordered_loops)
    {
      if (mg[loop_descriptor].offset_distance > largest_offset)
      {
        largest_offset = mg[loop_descriptor].offset_distance;
        deepest_loop = loop_descriptor;
      }
    }
    ROS_INFO_COND(verbose_, "Moving loop at depth %f to ordered list.", largest_offset);
    moveLoopItem(deepest_loop, unordered_loops, ordered_loops);

    // Find child of deepest loop (TODO check for multiple children, should never happen!)
    // Move child to ordered
    ovd::MGVertex child;
    while (getChild(ordered_loops.back(), mg, child))
    {
      if (exists(child, unordered_loops))
      {
        ROS_INFO_COND(verbose_, "Moving loop at depth %f to ordered list.", mg[child].offset_distance);
        moveLoopItem(child, unordered_loops, ordered_loops);
      }
      else
      {
        break;
      }
    }
  }

  // Convert ovg::OffsetLoops to polygons and discretize
  BOOST_FOREACH(ovd::MGVertex loop_descriptor, ordered_loops)
  {
    PolygonBoundary polygon;
    const ovd::OffsetLoop &loop = mg[loop_descriptor];
    ovd::OffsetVertex prior_vtx = loop.vertices.front();
    std::list<ovd::OffsetVertex>::const_iterator vtx=boost::next(loop.vertices.begin());
    while ( vtx!=loop.vertices.end() )
    {
        discretizeSegment(prior_vtx, *vtx, polygon);
        prior_vtx = *vtx;
        ++vtx;
    }
//    --vtx;
//    polygon.push_back(PolygonPt(vtx->p.x, vtx->p.y));
    polygons.push_back(polygon);
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

  convertPolygonsToProcessPath(polygons, offset_depths);

  return true;
}

void ProcessPathGenerator::discretizeArc(const ovd::OffsetVertex &op1, const ovd::OffsetVertex &op2, PolygonBoundary &bnd) const
{
  PolygonPt p1(op1.p.x, op1.p.y), p2(op2.p.x, op2.p.y);
  PolygonPt c(op2.c.x, op2.c.y);
  bnd.push_back(p1);    // Add first point

  // Find angle from p1 to p2. Check direction of theta against known direction of arc.
  double theta = std::atan2( (p1-c).cross(p2-c), (p1-c).dot(p2-c) );
  if (theta<0. && !op2.cw)
  {
    theta += 2.*M_PI;
  }
  else if  (theta>0. && op2.cw)
  {
    theta -= 2.*M_PI;
  }

  double max_theta(max_discretization_distance_/op2.r);
  if (std::abs(theta) > max_theta)
  {
    size_t new_ptcnt = static_cast<size_t>(std::ceil(std::abs(theta/max_theta)) -1.);
    double disc_angle = theta/static_cast<double>(new_ptcnt+1.);
    Eigen::Vector3d start_pt(op1.p.x-op2.c.x, op1.p.y-op2.c.y, 1.);
    for (size_t ii=1; ii<=new_ptcnt; ++ii)
    {
      double angle = disc_angle * static_cast<double>(ii);      // what angle this point is added at
      Eigen::Matrix3d rotator;
      rotator << cos(angle), -sin(angle), op2.c.x, sin(angle), cos(angle), op2.c.y, 0, 0, 1;
      Eigen::Vector3d new_pt = rotator * start_pt;
      bnd.push_back(PolygonPt(new_pt(0), new_pt(1)));
    }
  }


}

void ProcessPathGenerator::discretizeLinear(const ovd::OffsetVertex &op1, const ovd::OffsetVertex &op2, PolygonBoundary &bnd) const
{
  //add [p1-p2) with interpolated points between
  PolygonPt p1(op1.p.x, op1.p.y), p2(op2.p.x, op2.p.y);
  bnd.push_back(p1);    // Add first point

  double sep = p1.dist(p2);
  if (sep > max_discretization_distance_)
  {
    size_t new_ptcnt = static_cast<size_t>(std::ceil(sep/max_discretization_distance_) - 1.);
    double disc_dist = sep/static_cast<double>(new_ptcnt+1.);
    for (size_t ii=1; ii<=new_ptcnt; ++ii)
    {
      bnd.push_back(p1 + (p2-p1)*(static_cast<double>(ii)*disc_dist/sep));
    }
  }
}

void ProcessPathGenerator::discretizeSegment(const ovd::OffsetVertex &op1, const ovd::OffsetVertex &op2, PolygonBoundary &bnd) const
{
  if (op2.r == -1)
  {
    discretizeLinear(op1, op2, bnd);
  }
  else
  {
    discretizeArc(op1, op2, bnd);
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
  boost::graph_traits<ovd::MachiningGraph>::out_edge_iterator edge, edge_end;
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
  to.push_back(item);
  MachiningLoopList::iterator iter;
  for (iter=from.begin(); iter !=from.end(); ++iter)
  {
    if (item == *iter)
    {
      from.erase(iter);
      return;
    }
  }
  ROS_ASSERT(iter != from.end());
}

void operator<<(ProcessPt &pr_pt, const PolygonPt &pg_pt)
{
  pr_pt.setPosePosition(pg_pt.x, pg_pt.y, 0.);
}

} /* namespace godel_process_path */
