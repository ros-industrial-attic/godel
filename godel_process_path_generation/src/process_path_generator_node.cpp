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
 * process_path_generator_node.cpp
 *
 *  Created on: May 26, 2014
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <boost/tuple/tuple.hpp>
#include <godel_process_path_generation/utils.h>
#include <godel_process_path_generation/VisualizeBlendingPlan.h>
#include <godel_msgs/ProcessPlanningAction.h>
#include <godel_msgs/OffsetBoundary.h>
#include <godel_msgs/PathPlanning.h>
#include <godel_process_path_generation/polygon_pts.hpp>
#include <godel_process_path_generation/process_path_generator.h>
#include <godel_process_path_generation/process_path.h>
#include <godel_process_path_generation/polygon_utils.h>

const std::string OFFSET_POLYGON_SERVICE = "offset_polygon";

const static double DISCRETIZATION_DISTANCE = 0.01; // m
const static double TRAVERSE_HEIGHT = 0.075;        // m

double dist(const Eigen::Affine3d& from, const Eigen::Affine3d& to)
{
  return (from.translation() - to.translation()).norm();
}


bool pathDataToDurations(std::vector<ros::Duration>& times,
                         const std::vector<descartes::ProcessPt>& pts,
                         const std::vector<descartes::ProcessTransition>& transitions)
{
  times.clear();
  size_t n = transitions.size();
  if (0 == n)
  {
    ROS_WARN("No data to convert to timestamps.");
    return false;
  }
  ROS_ASSERT_MSG(pts.size() == n + 1, "Pts size %ld; Trans size %ld", n, pts.size());
  times.resize(n);
  ROS_INFO_STREAM("Creating " << n << " timestamps");

  for (size_t ii = 0; ii < n; ++ii)
  {
    descartes::LinearVelocityConstraintPtr vel = transitions.at(ii).getLinearVelocity();
    ROS_ASSERT_MSG(vel != NULL, "At point %ld", ii);
    times.at(ii) = ros::Duration(dist(pts.at(ii + 1).pose(), pts.at(ii).pose()) / vel->desired);
  }
  return true;
}


bool generateProcessPlan(descartes::ProcessPath& process_path,
                         const godel_msgs::PathPlanningRequest& req,
                         ros::ServiceClientPtr offset_service_client)
{
  // Create ProcessPathGenerator and initialize.
  godel_process_path::ProcessPathGenerator ppg;
  ppg.verbose_ = true;
  ppg.setDiscretizationDistance(DISCRETIZATION_DISTANCE);
  ppg.setMargin(req.params.margin);
  ppg.setOverlap(req.params.overlap);
  ppg.setToolRadius(req.params.tool_radius);
  ppg.setTraverseHeight(0.0); // Note: We added traverse height to the newer
                              // 'process_planning' component of our system
                              // so I set the param to zero here.
  if (!ppg.variables_ok())
  {
    ROS_ERROR("Cannot continue path generation with current variables.");
    return false;
  }

  // Call polygon_offset service.
  godel_msgs::OffsetBoundaryRequest ob_req;
  godel_msgs::OffsetBoundaryResponse ob_res;

  ob_req.discretization = DISCRETIZATION_DISTANCE;
  ob_req.initial_offset = req.params.tool_radius + req.params.margin;
  ob_req.offset_distance = req.params.tool_radius - req.params.overlap;
  ob_req.polygons = req.surface.boundaries;

  if (!offset_service_client->call(ob_req, ob_res))
  {
    ROS_ERROR("Bad response from %s", offset_service_client->getService().c_str());
    return false;
  }

  // Generate process paths.
  godel_process_path::PolygonBoundaryCollection paths;
  godel_process_path::utils::translations::geometryMsgsToGodel(paths, ob_res.offset_polygons);
  if (!ppg.setPathPolygons(&paths, &ob_res.offsets))
  {
    ROS_ERROR("Could not set polygon data in path planner.");
    return false;
  }
  if (!ppg.createProcessPath())
  {
    ROS_ERROR("Could not create process paths.");
    return false;
  }
  process_path = ppg.getProcessPath();

  return true;
}


bool pathGen(godel_msgs::PathPlanningRequest& req,
             godel_msgs::PathPlanningResponse& res,
             ros::ServiceClientPtr offset_service_client)
{
  // Call function to generate process path.
  godel_msgs::PathPlanningRequest path_planninging_request;
  path_planninging_request.params = req.params;
  path_planninging_request.surface = req.surface;
  descartes::ProcessPath process_path;
  generateProcessPlan(process_path, path_planninging_request, offset_service_client);

  // Populate service response
  std::vector<descartes::ProcessPt> pts;
  std::vector<descartes::ProcessTransition> transitions;
  boost::tie(pts, transitions) = process_path.data();

  res.poses = process_path.asPoseArray();

  return true;
}



int main(int argc, char** argv)
{

  ros::init(argc, argv, "process_path_generator");
  ros::NodeHandle nh;

  // waiting for service
  while (!ros::service::waitForService(OFFSET_POLYGON_SERVICE, ros::Duration(10.0f)))
  {
    ROS_WARN_STREAM("Connecting to service '" << OFFSET_POLYGON_SERVICE << "'");
  }

  ros::ServiceClientPtr boundary_offset_client(
      new ros::ServiceClient(nh.serviceClient<godel_msgs::OffsetBoundary>(OFFSET_POLYGON_SERVICE)));

  ros::ServiceServer path_generator =
      nh.advertiseService<godel_msgs::PathPlanningRequest, godel_msgs::PathPlanningResponse>(
          "process_path_generator", boost::bind(pathGen, _1, _2, boundary_offset_client));
  ROS_INFO("%s ready to service requests.", path_generator.getService().c_str());
  ros::spin();

  return 0;
}
