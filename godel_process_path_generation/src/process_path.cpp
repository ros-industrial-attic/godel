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
 * process_path.cpp
 *
 *  Created on: May 13, 2014
 *      Author: Dan Solomon
 */

#include "godel_process_path_generation/process_path.h"
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <eigen_conversions/eigen_msg.h>

namespace descartes
{

visualization_msgs::Marker ProcessPath::asMarker() const
{
  visualization_msgs::Marker marker;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0.;
  marker.scale.x = .002; /*1mm line width*/
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  for (std::vector<ProcessPt>::const_iterator pt = pts_.begin(); pt != pts_.end(); ++pt)
  {
    geometry_msgs::Point marker_pt;
    marker_pt.x = pt->pose().translation()(0);
    marker_pt.y = pt->pose().translation()(1);
    marker_pt.z = pt->pose().translation()(2);

    marker.points.push_back(marker_pt);
  }

  std_msgs::ColorRGBA red;
  red.a = 1.f;
  red.r = 1.f;
  red.g = red.b = 0.f;
  marker.colors = std::vector<std_msgs::ColorRGBA>(marker.points.size(), red);

  return marker;
}

geometry_msgs::PoseArray ProcessPath::asPoseArray() const
{
  geometry_msgs::PoseArray poses;
  for (const auto& pt : pts_)
  {
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    pose.position.x = pt.pose().translation()(0);
    pose.position.y = pt.pose().translation()(1);
    pose.position.z = pt.pose().translation()(2);

    poses.poses.push_back(pose);
  }

  return poses;
}

} /* namespace descartes */
