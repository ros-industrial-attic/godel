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
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Pose.h>
#include "godel_process_path_generation/VisualizeBlendingPlan.h"
#include "godel_process_path_generation/utils.h"

const float TOOL_DIA = .050;
const float TOOL_THK = .005;
const float TOOL_SHAFT_DIA = .006;
const float TOOL_SHAFT_LEN = .045;
const float MARGIN = .005;
const float PATH_OVERLAP = .01;
const float DISCRETIZATION = .0025;

visualization_msgs::MarkerArray toolMarker(const geometry_msgs::Point& pos,
                                           const geometry_msgs::Pose& pose)
{
  visualization_msgs::MarkerArray tool;
  tool.markers.resize(2);
  visualization_msgs::Marker& disk = tool.markers.at(0);
  visualization_msgs::Marker& shaft = tool.markers.at(1);

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
  disk.pose = pose;
  // disk/shaft position filled out below
  disk.type = visualization_msgs::Marker::CYLINDER;
  shaft = disk;

  tf::Transform marker_pose;
  tf::poseMsgToTF(pose, marker_pose);

  disk.id = 0;
  tf::Vector3 marker_pos(pos.x, pos.y, pos.z + .5 * TOOL_THK);
  marker_pos = marker_pose * marker_pos;
  tf::pointTFToMsg(marker_pos, disk.pose.position);
  disk.scale.x = disk.scale.y = TOOL_DIA;
  disk.scale.z = TOOL_THK;

  shaft.id = 1;
  marker_pos = tf::Vector3(pos.x, pos.y, pos.z + TOOL_THK + 0.5 * TOOL_SHAFT_LEN);
  marker_pos = marker_pose * marker_pos;
  tf::pointTFToMsg(marker_pos, shaft.pose.position);
  shaft.scale.x = shaft.scale.y = TOOL_SHAFT_DIA;
  shaft.scale.z = TOOL_SHAFT_LEN;

  return tool;
}

int main(int argc, char** argv)
{
  boost::program_options::options_description desc(
      "This node publishes a process path and animates a tool moving along it.");
  desc.add_options()("help", "produce help message")(
      "demo", "run in demo-mode (does not contact other nodes)");

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
  ros::ServiceClient path_generator_client =
      nh.serviceClient<godel_process_path_generation::VisualizeBlendingPlan>(
          "visualize_path_generator");
  ros::Publisher path_pub = nh.advertise<visualization_msgs::MarkerArray>("process_path", 1, true);
  ros::Publisher tool_pub = nh.advertise<visualization_msgs::MarkerArray>("sanding_tool", 1, true);
  ROS_INFO_STREAM("Started node '" << ros::this_node::getName() << "'");

  godel_process_path_generation::VisualizeBlendingPlanRequest bp_req;
  godel_process_path_generation::VisualizeBlendingPlanResponse bp_res;

  // Blending Velocity
  bp_req.params.approach_spd = .005;
  bp_req.params.blending_spd = .01;
  bp_req.params.retract_spd = .02;
  bp_req.params.traverse_spd = .05;

  // Blending Misc.
  bp_req.params.discretization = DISCRETIZATION;

  // Blending Offsets
  bp_req.params.margin = MARGIN;
  bp_req.params.overlap = PATH_OVERLAP;
  bp_req.params.safe_traverse_height = .05;
  bp_req.params.tool_radius = TOOL_DIA / 2.;

  // Pose of pocket surface
  geometry_msgs::Pose pocket_pose_msg;

  // Populate request for boundary offsets
  std::vector<geometry_msgs::Polygon>& boundaries = bp_req.surface.boundaries;
  if (vm.count("demo"))
  {
    ROS_INFO("Running in demo mode");
    // Outer boundary
    geometry_msgs::Point32 pt;
    geometry_msgs::Polygon boundary;
    pt.x = 0.;
    pt.y = 0.;
    boundary.points.push_back(pt);
    pt.x = .5;
    pt.y = 0.;
    boundary.points.push_back(pt);
    pt.x = .5;
    pt.y = .5;
    boundary.points.push_back(pt);
    //    pt.x = .25; pt.y = .125; boundary.points.push_back(pt);
    pt.x = .25;
    pt.y = .45;
    boundary.points.push_back(pt);
    pt.x = 0.;
    pt.y = .5;
    boundary.points.push_back(pt);
    boundaries.push_back(boundary);

    // Inner boundary (can only use with .25,.45 not with .25,.125)
    boundary.points.clear();
    pt.x = .3;
    pt.y = .1;
    boundary.points.push_back(pt);
    pt.x = .2;
    pt.y = .1;
    boundary.points.push_back(pt);
    pt.x = .25;
    pt.y = .2;
    boundary.points.push_back(pt);
    boundaries.push_back(boundary);

    // Pose of pocket surface
    tf::Pose pocket_pose;
    pocket_pose.setIdentity();
    pocket_pose.setOrigin(tf::Vector3(.25, -.5, .03));
    tf::Quaternion orient;
    orient.setEulerZYX(.1, .2, .3);
    pocket_pose.setRotation(orient);
    tf::poseTFToMsg(pocket_pose, pocket_pose_msg);
  }
  else
  {
    ROS_ERROR("Only demo mode is implemented. Run node with --demo option.");
    return -1;
  }

  // waiting for service
  while (!path_generator_client.waitForExistence(ros::Duration(10)))
  {
    ROS_WARN_STREAM("Connecting to Service " << path_generator_client.getService());
  }

  // Call service to offset boundaries
  ROS_INFO("Calling path planning service.");
  if (path_generator_client.call(bp_req, bp_res))
  {
    ROS_INFO("Call returned successfully.");
  }
  else
  {
    ROS_ERROR("Call returned unsuccessfully.");
    return -1;
  }

  std_msgs::ColorRGBA yellow;
  yellow.a = 1.f;
  yellow.b = 0.f;
  yellow.r = yellow.g = 1.f;

  std_msgs::ColorRGBA green;
  green.a = 1.f;
  green.g = 1.f;
  green.r = green.b = 0.f;

  godel_process_path::PolygonBoundaryCollection pbc;
  godel_process_path::utils::translations::geometryMsgsToGodel(pbc, bp_req.surface.boundaries);
  BOOST_FOREACH (godel_process_path::PolygonBoundary& boundary, pbc)
  {
    boundary.push_back(boundary.front());
  }
  visualization_msgs::MarkerArray pocket_marker;
  godel_process_path::utils::translations::godelToVisualizationMsgs(pocket_marker, pbc, yellow,
                                                                    .001);
  pocket_marker.markers.push_back(bp_res.path);
  size_t marker_cnt(0);
  BOOST_FOREACH (visualization_msgs::Marker& marker, pocket_marker.markers)
  {
    marker.header.frame_id = "world";
    marker.header.seq = marker_cnt;
    marker.header.stamp = ros::Time::now();
    marker.id = marker_cnt;
    marker.lifetime = ros::Duration(0);
    marker.frame_locked = true;
    marker.ns = "boundary";
    marker.pose = pocket_pose_msg;
    ++marker_cnt;
  }

  // Set up initial colors and namespace for path marker
  visualization_msgs::Marker& path_marker = pocket_marker.markers.back();
  path_marker.ns = "path";
  path_marker.colors.at(0) = green;

  size_t idx = 0;
  double time_factor = .02;
  while (ros::ok() && idx < bp_res.sleep_times.size())
  {
    // Show path and tool
    path_pub.publish(pocket_marker);
    tool_pub.publish(toolMarker(path_marker.points.at(idx), pocket_pose_msg));

    // Wait for next point, and color current position green
    ros::Duration(bp_res.sleep_times.at(idx) * time_factor).sleep();
    path_marker.colors.at(idx + 1) = green;
    ++idx;
  }
  path_pub.publish(pocket_marker); // publish last point
  tool_pub.publish(toolMarker(path_marker.points.at(idx), pocket_pose_msg));
  ros::Duration(1.).sleep(); // let last message be sent before quitting

  return 0;
}
