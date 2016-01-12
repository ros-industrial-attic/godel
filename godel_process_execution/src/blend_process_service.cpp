#include <godel_process_execution/blend_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <godel_msgs/TrajectoryExecution.h>

#include "process_utils.h"
#include <boost/thread.hpp>

#include <ros/topic.h>

godel_process_execution::BlendProcessExecutionService::BlendProcessExecutionService(const std::string& name,
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  // Simulation Server
  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);
  // Trajectory Execution Service
  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>(real_name);
  // The generic process execution service
  server_ = nh.advertiseService<BlendProcessExecutionService,
                                godel_msgs::BlendProcessExecution::Request,
                                godel_msgs::BlendProcessExecution::Response>
            (name, &godel_process_execution::BlendProcessExecutionService::executionCallback, this);
}

bool godel_process_execution::BlendProcessExecutionService::executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                                                                         godel_msgs::BlendProcessExecution::Response& res)
{
  using simulator_service::SimulateTrajectory;

  if (req.simulate)
  {
    return simulateProcess(req);
  }
  else
  {
    // Real execution; if we shouldn't wait, spawn a thread and return this function immediately.
    if (req.wait_for_execution)
    {
      return executeProcess(req);
    }
    else
    {
      boost::thread(&godel_process_execution::BlendProcessExecutionService::executeProcess, this, req);
      return true;
    }
  }
}

bool godel_process_execution::BlendProcessExecutionService::executeProcess(godel_msgs::BlendProcessExecution::Request req)
{
  godel_msgs::TrajectoryExecution srv_approach;
  srv_approach.request.wait_for_execution = true;
  srv_approach.request.trajectory = req.trajectory_approach;

  godel_msgs::TrajectoryExecution srv_process;
  srv_process.request.wait_for_execution = true;
  srv_process.request.trajectory = req.trajectory_process;

  godel_msgs::TrajectoryExecution srv_depart;
  srv_depart.request.wait_for_execution = true;
  srv_depart.request.trajectory = req.trajectory_depart;

  if (!real_client_.call(srv_approach))
  {
    ROS_WARN("Execution client unavailable or unable to execute approach trajectory.");
    return false;
  }

  if (!real_client_.call(srv_process))
  {
    ROS_WARN("Execution client unavailable or unable to execute process trajectory.");
    return false;
  }

  if (!real_client_.call(srv_depart))
  {
    ROS_WARN("Execution client unavailable or unable to execute departure trajectory.");
    return false;
  }

  return true;
}

bool godel_process_execution::BlendProcessExecutionService::simulateProcess(godel_msgs::BlendProcessExecution::Request req)
{
  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = req.trajectory_approach;
  appendTrajectory(aggregate_traj, req.trajectory_process);
  appendTrajectory(aggregate_traj, req.trajectory_depart);

  // Pass the trajectory to the simulation service
  SimulateTrajectory srv;
  srv.request.wait_for_execution = req.wait_for_execution;
  srv.request.trajectory = aggregate_traj;

  // Call simulation service
  if (!sim_client_.call(srv))
  {
    ROS_WARN("Simulation client unavailable or unable to simulate trajectory.");
    return false;
  }
  else
  {
    return true;
  }
}
