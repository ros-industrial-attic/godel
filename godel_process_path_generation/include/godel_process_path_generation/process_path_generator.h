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
 *      Author: ros
 */

#ifndef PROCESS_PATH_GENERATOR_H_
#define PROCESS_PATH_GENERATOR_H_

#include "godel_process_path_generation/polygon_pts.hpp"
#include <openvoronoi/voronoidiagram.hpp>

namespace godel_process_path
{

class ProcessPathGenerator
{
public:
  ProcessPathGenerator(): vd_(new ovd::VoronoiDiagram(1,100)), tool_radius_(0.), margin_(0.), overlap_(0.) {};
  virtual ~ProcessPathGenerator() {};

  bool configure(PolygonBoundaryCollection boundaries);
  bool createProcessPath();

  void setMargin(double margin) {margin_=margin;}
  void setOverlap(double overlap) {overlap_=overlap;}
  void setToolRadius(double radius) {tool_radius_=radius;}


private:

  /*ovd::VoronoiDiagram* vd = new ovd::VoronoiDiagram(1,100); // (r, bins)
   * double r: radius of circle within which all input geometry must fall. use 1 (unit-circle). Scale geometry if necessary.
   * int bins:  bins for face-grid search. roughly sqrt(n), where n is the number of sites is good according to Held. */
   boost::shared_ptr<ovd::VoronoiDiagram> vd_;

   double tool_radius_; /**<Tool radius(m) used for offsetting paths */
   double margin_;      /**<Margin (m) around boundary to leave untouched (first pass only) */
   double overlap_;     /**<Amount of overlap(m) between adjacent passes. */

   bool variables_ok()
   {
     return  tool_radius_ >= 0. &&
             margin_ >= 0. &&
             overlap_ < 2.*tool_radius_;
   }
};

} /* namespace godel_process_path */
#endif /* PROCESS_PATH_GENERATOR_H_ */
