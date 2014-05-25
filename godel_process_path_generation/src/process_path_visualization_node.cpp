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
 * process_path_visualization_node.cpp
 *
 *  Created on: May 14, 2014
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/next_prior.hpp>
#include <visualization_msgs/MarkerArray.h>
#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/process_path.h"
#include "godel_process_path_generation/mesh_importer.h"
#include "godel_process_path_generation/process_path_generator.h"
#include "godel_msgs/OffsetBoundary.h"
#include "godel_process_path_generation/utils.h"


using descartes::ProcessPt;
using descartes::ProcessTransition;
using godel_process_path::PolygonPt;
using godel_process_path::PolygonBoundary;
using godel_process_path::PolygonBoundaryCollection;

const float TOOL_DIA = .050;
const float TOOL_THK = .005;
const float TOOL_SHAFT_DIA = .006;
const float TOOL_SHAFT_LEN = .045;
const float MARGIN = .005;
const float PATH_OVERLAP = .01;
const float DISCRETIZATION = .0025;

double dist(const Eigen::Affine3d &from, const Eigen::Affine3d &to)
{
  return (from.translation()-to.translation()).norm();
}

std::vector<double> pathDataToTimestamps(const std::vector<ProcessPt> &pts, const std::vector<ProcessTransition> &transitions)
{
  size_t n=transitions.size();
  if (0==n)
  {
    ROS_WARN("No data to convert to timestamps.");
    return std::vector<double>(0);
  }
  ROS_ASSERT_MSG(pts.size() == n+1, "Pts size %ld; Trans size %ld", n, pts.size());
  std::vector<double> timestamps(n);
  ROS_INFO_STREAM("Creating " << n << " timestamps");

  for (size_t ii=0; ii<n; ++ii)
  {
    descartes::LinearVelocityConstraintPtr vel = transitions.at(ii).getLinearVelocity();
    ROS_ASSERT_MSG(vel != NULL, "At point %ld", ii);
    timestamps.at(ii) = dist(pts.at(ii+1).pose(), pts.at(ii).pose()) / vel->desired;
//    std::cout << dist(pts.at(ii+1).pose(), pts.at(ii).pose()) << "/" << vel->desired << "/" << timestamps.at(ii) << std::endl;
  }
  return timestamps;
}

visualization_msgs::MarkerArray toolMarker(double x, double y, double z)
{
  visualization_msgs::MarkerArray tool;
  tool.markers.resize(2);
  visualization_msgs::Marker &disk = tool.markers.at(0);
  visualization_msgs::Marker &shaft = tool.markers.at(1);

  std_msgs::ColorRGBA blue;
  blue.r = 0.;
  blue.g = .1;
  blue.b = 1.;
  blue.a = 0.7;

  disk.action = visualization_msgs::Marker::ADD;
  disk.color = blue;
  disk.frame_locked = true;
  disk.header.frame_id = "world";
  disk.header.seq = 0;
  disk.header.stamp = ros::Time::now();
  disk.lifetime = ros::Duration(0.);
  disk.pose.position.x = x;
  disk.pose.position.y = y;
  disk.pose.orientation.x = disk.pose.orientation.y = disk.pose.orientation.z = 0.;
  disk.pose.orientation.w = 1.;
  disk.type = visualization_msgs::Marker::CYLINDER;
  shaft = disk;

  disk.id = 0;
  disk.pose.position.z = z + .5*TOOL_THK;
  disk.scale.x = disk.scale.y = TOOL_DIA;
  disk.scale.z = TOOL_THK;

  shaft.id = 1;
  shaft.pose.position.z = z + TOOL_THK + 0.5*TOOL_SHAFT_LEN;
  shaft.scale.x = shaft.scale.y = TOOL_SHAFT_DIA;
  shaft.scale.z = TOOL_SHAFT_LEN;

  return tool;
}

int main(int argc, char **argv)
{
  boost::program_options::options_description desc("This node publishes a process path and animates a tool moving along it.");
  desc.add_options()
      ("help", "produce help message")
      ("demo",  "run in demo-mode (does not contact other nodes)")
  ;

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);


  if (vm.count("help"))
  {
      std::cout << desc << "\n";
      return 0;
  }

  ros::init(argc, argv, "process_path_visualization");
  ros::NodeHandle nh;
  ros::ServiceClient boundary_offset_client=nh.serviceClient<godel_msgs::OffsetBoundary>("offset_polygon");
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("process_path", 1, true);
  ros::Publisher tool_pub = nh.advertise<visualization_msgs::MarkerArray>("sanding_tool", 1, true);
  ROS_INFO_STREAM("Started node '" << ros::this_node::getName() << "'");

  godel_process_path::ProcessPathGenerator ppg;
  ppg.verbose_ = true;
  ppg.setTraverseHeight(.05);
  ppg.setMargin(MARGIN);
  ppg.setOverlap(PATH_OVERLAP);
  ppg.setToolRadius(TOOL_DIA/2.);
  godel_process_path::ProcessVelocity vel;
  vel.approach = .005;
  vel.blending = .01;
  vel.retract = .02;
  vel.traverse = .05;
  ppg.setVelocity(vel);
  ppg.setDiscretizationDistance(DISCRETIZATION);

  // Populate request for boundary offsets
  godel_msgs::OffsetBoundaryRequest boundary_offset_request;
  boundary_offset_request.discretization = DISCRETIZATION;
  boundary_offset_request.initial_offset = TOOL_DIA/2. + MARGIN;
  boundary_offset_request.offset_distance = TOOL_DIA/2. - PATH_OVERLAP;
  std::vector<geometry_msgs::Polygon> &boundaries = boundary_offset_request.polygons;
  if (vm.count("demo"))
  {
    ROS_INFO("Running in demo mode");
    // Outer boundary
    geometry_msgs::Point32 pt;
    geometry_msgs::Polygon boundary;
    pt.x = 0.; pt.y = 0.; boundary.points.push_back(pt);
    pt.x = .5; pt.y = 0.; boundary.points.push_back(pt);
    pt.x = .5; pt.y = .5; boundary.points.push_back(pt);
//    pt.x = .25; pt.y = .125; boundary.points.push_back(pt);
    pt.x = .25; pt.y = .45; boundary.points.push_back(pt);
    pt.x = 0.; pt.y = .5; boundary.points.push_back(pt);
    boundaries.push_back(boundary);

    // Inner boundary (can only use with .25,.45 not with .25,.125)
    boundary.points.clear();
    pt.x = .3; pt.y = .1; boundary.points.push_back(pt);
    pt.x = .2; pt.y = .1; boundary.points.push_back(pt);
    pt.x = .25; pt.y = .2; boundary.points.push_back(pt);
    boundaries.push_back(boundary);
  }
  else
  {
    ROS_ERROR("Only demo mode is implemented. Run node with --demo option.");
    return -1;
  }

  // Call service to offset boundaries
  godel_msgs::OffsetBoundaryResponse boundary_offset_response;
  ROS_INFO("Calling offset service.");
  if (boundary_offset_client.call(boundary_offset_request, boundary_offset_response))
  {
    ROS_INFO("Call returned successfully.");
  }
  else
  {
    ROS_ERROR("Call returned unsuccessfully.");
    return -1;
  }

  godel_process_path::PolygonBoundaryCollection pbc;
  godel_process_path::utils::translations::geometryMsgsToGodel(pbc, boundary_offset_response.offset_polygons);

  ppg.setPathPolygons(&pbc, &boundary_offset_response.offsets);
  if (!ppg.createProcessPath())
  {
    ROS_ERROR("Could not create process path");
    return 0;
  }
  const descartes::ProcessPath &process_path = ppg.getProcessPath();
  const std::pair<std::vector<ProcessPt>, std::vector<ProcessTransition> > &data = process_path.data();
  std::vector<double> timestamps = pathDataToTimestamps(data.first, data.second);

  visualization_msgs::Marker marker = process_path.asMarker();
  marker.header.frame_id = "world";
  marker.header.seq = 0;
  marker.header.stamp = ros::Time::now();
  marker.id = 0;
  marker.lifetime = ros::Duration(0);
  marker.frame_locked = true;
  marker.ns = "";

  std_msgs::ColorRGBA green;
  green.a = 1.f;
  green.g = 1.f;
  green.r = green.b = 0.f;
  marker.colors.at(0) = green;

  size_t idx = 0;
  double time_factor = .05;
  while (ros::ok() && idx < timestamps.size())
  {
    path_pub.publish(marker);
    geometry_msgs::Point &pt = marker.points.at(idx);
    tool_pub.publish(toolMarker(pt.x, pt.y, pt.z));
//    std::cout << "Waiting " << timestamps.at(idx) << std::endl;
    ros::Duration(timestamps.at(idx)*time_factor).sleep();
    marker.colors.at(idx+1) = green;
    ++idx;
  }
  path_pub.publish(marker);     // publish last point
  ros::Duration(1.).sleep();

  return 1;
}

