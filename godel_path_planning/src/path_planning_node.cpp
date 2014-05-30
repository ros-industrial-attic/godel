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
 * path_planning_node.cpp
 *
 *  Created on: May 29, 2014
 *      Author: Dan Solomon
 */


#include <ros/ros.h>
#include <boost/bind.hpp>
#include <boost/foreach.hpp>
#include "godel_msgs/BlendingPlan.h"
#include "godel_msgs/ProcessPlanning.h"
#include "godel_process_path_generation/process_path_generator.h"


bool path_planner_cb(godel_msgs::BlendingPlanRequest &req, godel_msgs::BlendingPlanResponse &res,
                     godel_process_path::ProcessPathGenerator *ppg,
                     ros::ServiceClientPtr process_planner_client, ros::ServiceClientPtr move_group_client)
{
  ROS_WARN("Path planner callback not implemented yet!");
  return false;

  std::vector<trajectory_msgs::JointTrajectory> trajectories;
  BOOST_FOREACH(const godel_msgs::SurfaceBoundaries &surface, req.surfaces)
  {
    // Call process planner to get process plan
    godel_msgs::ProcessPlanning process_planner_srv;
    process_planner_srv.request.params = req.params;
    process_planner_srv.request.surface.boundaries = surface.boundaries;
    process_planner_client->call(process_planner_srv);

    // Call trajectory planner to get trajectory
    //TODO trajectories.push_back(trajectory_planning.result.trajectory);
  }
  res.trajectory.points.clear();
  //TODO fill out joint trajectory names, header, etc.
  for (std::vector<trajectory_msgs::JointTrajectory>::iterator trajectory=boost::next(trajectories.begin());
       trajectory!=trajectories.end(); ++trajectory)
  {
    //stitcher_srv
    //sticher_srv.request.from_pt = boost::prior(trajectory)->back();
    //sticher_srv.request.to_pt = trajectory->front();
    //move_group_client->call(stitcher_srv);
    //res.trajectory.points.insert(res.trajectory.points.end(), stitcher_srv.res.trajectory.begin(), stitcher_srv.res.trajectory.end());
  }
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "godel_path_planner");
  ros::NodeHandle nh;

  godel_process_path::ProcessPathGenerator ppg;
  ros::ServiceClientPtr process_path_generation_client(new ros::ServiceClient(nh.serviceClient<godel_msgs::ProcessPlanning>("process_path_generator")));
  ros::ServiceClientPtr move_group_client; // TODO add client for move_group for "stitching" trajectories together

  ros::ServiceServer path_planner_service = nh.advertiseService<godel_msgs::BlendingPlanRequest,
                                                                godel_msgs::BlendingPlanResponse>
                                            ("path_planner",
                                             boost::bind(path_planner_cb, _1, _2, &ppg, process_path_generation_client, move_group_client));

  ROS_INFO("%s ready to service requests.", path_planner_service.getService().c_str());

  ros::spin();

  return 0;
}
