#include <godel_process_execution/keyence_process_service.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include "godel_msgs/TrajectoryExecution.h"

#include "process_utils.h"

#include <boost/thread.hpp>

#include <ros/topic.h>

const static int KEYENCE_PROGRAM_LASER_ON = 4;
const static int KEYENCE_PROGRAM_LASER_OFF = 0;

const static std::string KEYENCE_PROGRAM_SERVICE_NAME = "change_program";
const static std::string EXECUTION_SERVICE_NAME = "execute_path";
const static std::string SIMULATION_SERVICE_NAME = "simulate_path";
const static std::string SERVICE_SERVER_NAME = "scan_process_execution";


godel_process_execution::KeyenceProcessService::KeyenceProcessService(ros::NodeHandle& nh)
{
  // Connect to motion servers and I/O server
  sim_client_ = nh.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>(SIMULATION_SERVICE_NAME);

  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>(EXECUTION_SERVICE_NAME);

  // Create this process execution server
  server_ = nh.advertiseService<KeyenceProcessService,
                                godel_msgs::KeyenceProcessExecution::Request,
                                godel_msgs::KeyenceProcessExecution::Response>
            (SERVICE_SERVER_NAME, &godel_process_execution::KeyenceProcessService::executionCallback, this);
}

bool godel_process_execution::KeyenceProcessService::executionCallback(godel_msgs::KeyenceProcessExecution::Request& req,
                                                                                godel_msgs::KeyenceProcessExecution::Response& res)
{
  if (req.simulate)
  {
    return simulateProcess(req);
  }
  else
  {
    if (req.wait_for_execution)
    {
      return executeProcess(req);
    }
    else
    {
      boost::thread(&godel_process_execution::KeyenceProcessService::executeProcess, this, req);
      return true;
    }
  }
}

bool godel_process_execution::KeyenceProcessService::executeProcess(godel_msgs::KeyenceProcessExecution::Request& req)
{
  ROS_WARN_STREAM("Keyence driver is unimplemented.");
  return false;
}

bool godel_process_execution::KeyenceProcessService::simulateProcess(godel_msgs::KeyenceProcessExecution::Request& req)
{
  using industrial_robot_simulator_service::SimulateTrajectory;

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
    ROS_ERROR("Simulation client unavailable or unable to simulate trajectory.");
    return false;
  }
  else
  {
    return true;
  }
}
