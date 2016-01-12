#include <godel_process_execution/keyence_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include "godel_msgs/TrajectoryExecution.h"

#include "process_utils.h"

#include <boost/thread.hpp>

#include "keyence_driver/ChangeProgram.h"

#include <ros/topic.h>

const static int KEYENCE_PROGRAM_LASER_ON = 4;
const static int KEYENCE_PROGRAM_LASER_OFF = 0;

const static std::string KEYENCE_PROGRAM_SERVICE_NAME = "change_program";

godel_process_execution::KeyenceProcessExecutionService::KeyenceProcessExecutionService(const std::string& name, 
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  // Connect to motion servers and I/O server
  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>(real_name);

  keyence_client_ = nh.serviceClient<keyence_driver::ChangeProgram>(KEYENCE_PROGRAM_SERVICE_NAME);

  // Create this process execution server
  server_ = nh.advertiseService<KeyenceProcessExecutionService,
                                godel_msgs::KeyenceProcessExecution::Request,
                                godel_msgs::KeyenceProcessExecution::Response>
            (name, &godel_process_execution::KeyenceProcessExecutionService::executionCallback, this);
}

bool godel_process_execution::KeyenceProcessExecutionService::executionCallback(godel_msgs::KeyenceProcessExecution::Request& req,
                                                                                godel_msgs::KeyenceProcessExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;

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
      boost::thread(&godel_process_execution::KeyenceProcessExecutionService::executeProcess, this, req);
      return true;
    }
  }
}

bool godel_process_execution::KeyenceProcessExecutionService::executeProcess(godel_msgs::KeyenceProcessExecution::Request& req)
{
  // Check for keyence existence
  if (!keyence_client_.exists())
  {
    ROS_WARN_STREAM("Keyence ROS server is not available on service " << keyence_client_.getService());
    return false;
  }

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

  keyence_driver::ChangeProgram keyence_srv;
  keyence_srv.request.program_no = KEYENCE_PROGRAM_LASER_ON;

  if (!keyence_client_.call(keyence_srv))
  {
    ROS_WARN_STREAM("Unable to activate keyence (program " << KEYENCE_PROGRAM_LASER_ON << ").");
     return false;
  }

  if (!real_client_.call(srv_process))
  {
    ROS_WARN("Execution client unavailable or unable to execute process trajectory.");
    return false;
  }

  // Turn keyence off
  keyence_srv.request.program_no = KEYENCE_PROGRAM_LASER_OFF;
  if (!keyence_client_.call(keyence_srv))
  {
    ROS_WARN_STREAM("Unable to de-activate keyence (program " << KEYENCE_PROGRAM_LASER_OFF << ").");
    return false;
  }

  if (!real_client_.call(srv_depart))
  {
    ROS_WARN("Execution client unavailable or unable to execute departure trajectory.");
    return false;
  }

  return true;
}

bool godel_process_execution::KeyenceProcessExecutionService::simulateProcess(godel_msgs::KeyenceProcessExecution::Request& req)
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
