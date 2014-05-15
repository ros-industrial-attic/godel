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
#include <godel_process_path_generation/polygon_pts.hpp>
#include "godel_process_path_generation/process_path.h"
#include "godel_process_path_generation/mesh_importer.h"
#include "godel_process_path_generation/process_path_generator.h"
#include <boost/foreach.hpp>
#include <boost/next_prior.hpp>


using descartes::ProcessPt;
using descartes::ProcessTransition;
using godel_process_path::PolygonPt;
using godel_process_path::PolygonBoundary;
using godel_process_path::PolygonBoundaryCollection;


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
      return 1;
  }

  ros::init(argc, argv, "process_path_visualization");
  ros::NodeHandle nh;
  ros::Publisher path_pub = nh.advertise<visualization_msgs::Marker>("process_path", 1, true);
  ROS_INFO_STREAM("Started node '" << ros::this_node::getName() << "'");

  godel_process_path::ProcessPathGenerator ppg;
  ppg.setTraverseHeight(.05);
  ppg.setMargin(.005);
  ppg.setOverlap(.01);
  ppg.setToolRadius(.025);
  godel_process_path::ProcessVelocity vel;
  vel.approach = .005;
  vel.blending = .01;
  vel.retract = .02;
  vel.traverse = .05;
  ppg.setVelocity(vel);
  ppg.setDiscretizationDistance(.0025);

  PolygonBoundary boundary;
  if (vm.count("demo"))
  {
    ROS_INFO("Running in demo mode");
    boundary.push_back(PolygonPt(0., 0.));
    boundary.push_back(PolygonPt(.5, 0.));
    boundary.push_back(PolygonPt(.5, .5));
    boundary.push_back(PolygonPt(.25, .125));
    boundary.push_back(PolygonPt(.0, .5));
  }
  else
  {

  }
  ppg.configure(PolygonBoundaryCollection(1,boundary));
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
  double time_factor = .2;
  while (ros::ok() && idx < timestamps.size())
  {
    path_pub.publish(marker);
    std::cout << "Waiting " << timestamps.at(idx) << std::endl;
    ros::Duration(timestamps.at(idx)*time_factor).sleep();
    marker.colors.at(idx+1) = green;
    ++idx;
  }
  path_pub.publish(marker);     // publish last point
  ros::Duration(1.).sleep();

  return 1;
}

