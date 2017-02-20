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
 * process_path_generator.h
 *
 *  Created on: May 9, 2014
 *      Author: Dan Solomon
 */

#ifndef PROCESS_PATH_GENERATOR_H_
#define PROCESS_PATH_GENERATOR_H_

#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/process_path.h"
#include "godel_process_path_generation/polygon_utils.h"

using descartes::ProcessPt;
using descartes::ProcessPath;

namespace godel_process_path
{

struct ProcessVelocity
{
  ProcessVelocity() : approach(0.), blending(0.), retract(0.), traverse(0.){};
  ProcessVelocity(double a, double b, double r, double t)
      : approach(a), blending(b), retract(r), traverse(t){};
  double approach, blending, retract, traverse;
};

class ProcessPathGenerator
{
public:
  ProcessPathGenerator()
      : tool_radius_(0.), margin_(0.), overlap_(0.), safe_traverse_height_(-1.), verbose_(false){};
  virtual ~ProcessPathGenerator(){};

  bool createProcessPath();
  const descartes::ProcessPath& getProcessPath() const { return process_path_; }

  /**@brief Set data used by path generator.
   * Note: Polygons data may become rotated during processing, but order will remain consistent.
   * @param polygons Pointer to collection of polygons that will be joined into ProcessPath
   * @param offset_depths Offset distance of each polygon
   */
  bool setPathPolygons(PolygonBoundaryCollection* polygons, std::vector<double>* offset_depths)
  {
    if (!polygon_utils::checkBoundaryCollection(*polygons))
    {
      ROS_WARN("Malformed polygons detected.");
      return false;
    }

    path_polygons_ = polygons;
    path_offsets_ = offset_depths;
    return true;
  }

  void setDiscretizationDistance(double d) { max_discretization_distance_ = std::abs(d); }
  void setMargin(double margin) { margin_ = margin; }
  void setOverlap(double overlap) { overlap_ = overlap; }
  void setToolRadius(double radius) { tool_radius_ = std::abs(radius); }
  void setTraverseHeight(double height) { safe_traverse_height_ = height; }
  void setVelocity(const ProcessVelocity& vel) { velocity_ = vel; }

  /**@brief Check if values of offset variables are acceptable */
  bool variables_ok() const
  {
    return tool_radius_ >= 0. &&                     /*tool must be real*/
           margin_ >= 0. &&                          /*negative margin is dangerous*/
           overlap_ < 2. * tool_radius_ &&           /*offset must increment inward*/
           (tool_radius_ != 0. || overlap_ != 0.) && /*offset must be positive*/
           safe_traverse_height_ >= 0.; /*negative traverse height may be inside part!*/
  }

  bool verbose_;

private:
  // TODO comment Does not add start/end
  void addInterpolatedProcessPts(const ProcessPt& start, const ProcessPt& end);

  // TODO comment
  void addPolygonToProcessPath(const PolygonBoundary& bnd);

  // TODO comment
  void addTraverseToProcessPath(const PolygonPt& from, const PolygonPt& to);

  /**@brief Create a ProcessTransition with linear velocity
   * ProcessTransition will be populated with linear velocity [0, vel, double::max()]
   * @param vel Desired path velocity
   * @return descartes::ProcessTransition
   */
  descartes::ProcessTransition velToTransition(double vel)
  {
    descartes::ProcessTransition pt;
    pt.setLinearVelocityConstraint(descartes::LinearVelocityConstraint(vel));
    return pt;
  }

  double tool_radius_; /**<Tool radius(m) used for offsetting paths */
  double margin_;      /**<Margin (m) around boundary to leave untouched (first pass only) */
  double overlap_;     /**<Amount of overlap(m) between adjacent passes. */
  double safe_traverse_height_; /**<Height to move to when traversing to new loops */

  double
      max_discretization_distance_; /**<(m) When discretizing segments, use this or less distance
                                       between points */

  PolygonBoundaryCollection* path_polygons_;
  const std::vector<double>* path_offsets_;

  descartes::ProcessPath process_path_;
  ProcessVelocity velocity_; /**<Velocities for different types of path movements */
};

} /* namespace godel_process_path */
#endif /* PROCESS_PATH_GENERATOR_H_ */
