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
#include <openvoronoi/voronoidiagram.hpp>

namespace godel_process_path
{

bool ProcessPathGenerator::configure(PolygonBoundaryCollection boundaries)
{
  ovd::VoronoiDiagram* vd = new ovd::VoronoiDiagram(1,100); // (r, bins)
  // double r: radius of circle within which all input geometry must fall. use 1 (unit-circle). Scale geometry if necessary.
  // int bins:  bins for face-grid search. roughly sqrt(n), where n is the number of sites is good according to Held.

  for (PolygonBoundaryCollection::const_iterator boundary=boundaries.begin(), bs_end=boundaries.end(); boundary!=bs_end; ++boundary)
  {
    std::vector<int> pt_id;
    pt_id.reserve(boundary->size());
    bool first=true;
    for (PolygonBoundary::const_iterator pt=boundary->begin(), b_end=boundary->end(); pt!=b_end; ++pt)
    {
      int id = vd->insert_point_site(ovd::Point(pt->x, pt->y));
      if (first)
      {
        first=false;
      }
      else
      {
        vd->insert_line_site(pt_id.back(), id);
      }
      pt_id.push_back(id);
    }
    vd->insert_line_site(pt_id.back(), pt_id.front());

  }
  if (!vd->check())
  {
    ROS_ERROR("Voroni diagram check failed.");
    return false;
  }

  ovd::polygon_interior_filter pi(true);
  vd->filter(&pi);

  return true;
}

} /* namespace godel_process_path */
