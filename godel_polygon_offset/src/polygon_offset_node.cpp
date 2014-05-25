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

using godel_polygon_offset::OffsetPolygonRequest;
using godel_polygon_offset::OffsetPolygonResponse;


void godelToGeometryMsgs(std::vector<geometry_msgs::Polygon> &polygons_msg, const godel_process_path::PolygonBoundaryCollection &pbc)
{
  polygons_msg.clear();
  BOOST_FOREACH(godel_process_path::PolygonBoundary polygon, pbc)
  {
    geometry_msgs::Polygon polygon_msg;
    BOOST_FOREACH(godel_process_path::PolygonPt pt, polygon)
    {
      geometry_msgs::Point32 pt_msg;
      pt_msg.x = pt.x;
      pt_msg.y = pt.y;
      polygon_msg.points.push_back(pt_msg);
    }
    polygons_msg.push_back(polygon_msg);
  }
}

void geometryMsgsToGodel(godel_process_path::PolygonBoundaryCollection &pbc, const std::vector<geometry_msgs::Polygon> &polygons_msg)
{
  pbc.clear();
  BOOST_FOREACH(geometry_msgs::Polygon polygon_msg, polygons_msg)
  {
    godel_process_path::PolygonBoundary polygon;
    BOOST_FOREACH(geometry_msgs::Point32 pt, polygon_msg.points)
    {
      polygon.push_back(godel_process_path::PolygonPt(pt.x, pt.y));
    }
    pbc.push_back(polygon);
  }
}

bool offset_polygons_cb(OffsetPolygonRequest &req, OffsetPolygonResponse &res)
{
  godel_polygon_offset::PolygonOffset po;
  godel_process_path::PolygonBoundaryCollection pbc;
  geometryMsgsToGodel(pbc, req.polygons);
  po.init(pbc, req.offset_distance, req.initial_offset, req.discretization);
  po.generateOrderedOffsets(pbc, res.offsets);
  godelToGeometryMsgs(res.offset_polygons, pbc);
  return false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "polygon_offset_node");
  ros::NodeHandle nh;
  ros::ServiceServer service = nh.advertiseService("offset_polygon", offset_polygons_cb);
  ros::spin();
  return 0;
}
