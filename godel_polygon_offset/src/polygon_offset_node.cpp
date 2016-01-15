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
 * polygon_offset_node.cpp
 *
 *  Created on: May 24, 2014
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <boost/foreach.hpp>
#include "godel_polygon_offset/OffsetPolygon.h"
#include "godel_polygon_offset/polygon_offset.h"
#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/utils.h"

using godel_polygon_offset::OffsetPolygonRequest;
using godel_polygon_offset::OffsetPolygonResponse;
using namespace godel_process_path;

bool offset_polygons_cb(OffsetPolygonRequest& req, OffsetPolygonResponse& res)
{
  godel_polygon_offset::PolygonOffset po;
  po.verbose_ = true;

  godel_process_path::PolygonBoundaryCollection pbc;
  utils::translations::geometryMsgsToGodel(pbc, req.polygons);
  ROS_INFO_STREAM("Received request with " << pbc.size() << " boundary polygons.");

  if (req.initial_offset <= 0.)
  {
    req.initial_offset = req.offset_distance; // Default initial offset if unspecified.
  }
  if (!po.init(pbc, req.offset_distance, req.initial_offset, req.discretization))
  {
    ROS_ERROR("Could not initialize PolygonOffset.");
    return false;
  }
  if (!po.generateOrderedOffsets(pbc, res.offsets)) /* Generates polygons and offset list*/
  {
    ROS_ERROR("Could not offset boundaries.");
    return false;
  }
  utils::translations::godelToGeometryMsgs(res.offset_polygons, pbc);
  ROS_INFO_STREAM("Returning " << pbc.size() << " offset polygons.");
  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "polygon_offset_node");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("offset_polygon", offset_polygons_cb);
  ROS_INFO("%s ready to service requests.", service.getService().c_str());
  ros::spin();
  return 0;
}
