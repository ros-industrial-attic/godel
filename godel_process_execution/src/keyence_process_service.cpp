#include <godel_process_execution/keyence_process_service.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include "godel_msgs/TrajectoryExecution.h"
#include <godel_utils/ensenso_guard.h>
#include "keyence_experimental/ChangeProgram.h"
#include <std_srvs/Trigger.h>

#include "process_utils.h"

#include <boost/thread.hpp>

#include <ros/topic.h>

const static int KEYENCE_PROGRAM_LASER_ON = 1;
const static int KEYENCE_PROGRAM_LASER_OFF = 0;

const static std::string KEYENCE_PROGRAM_SERVICE_NAME = "change_program";
const static std::string EXECUTION_SERVICE_NAME = "path_execution";
const static std::string SIMULATION_SERVICE_NAME = "simulate_path";
const static std::string SERVICE_SERVER_NAME = "scan_process_execution";
const static std::string RESET_SCANS_SERVICE = "reset_scan_server";
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "scan_process_execution_as";

godel_process_execution::KeyenceProcessService::KeyenceProcessService(ros::NodeHandle& nh) : nh_(nh),
  process_exe_action_server_(nh_,
                           PROCESS_EXE_ACTION_SERVER_NAME,
                           boost::bind(&godel_process_execution::KeyenceProcessService::executionCallback, this, _1),
                           false)
{
  // Connect to motion servers and I/O server
  sim_client_ = nh.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>(
      SIMULATION_SERVICE_NAME);

  real_client_ = nh.serviceClient<godel_msgs::TrajectoryExecution>(EXECUTION_SERVICE_NAME);

  keyence_client_ = nh.serviceClient<keyence_experimental::ChangeProgram>(KEYENCE_PROGRAM_SERVICE_NAME);

  // For reseting the scan server
  reset_scan_server_ = nh.serviceClient<std_srvs::Trigger>(RESET_SCANS_SERVICE);

  // start the action server
  process_exe_action_server_.start();

}

void godel_process_execution::KeyenceProcessService::executionCallback(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  godel_msgs::ProcessExecutionResult res;
  if (goal->simulate)
  {
    res.success = simulateProcess(goal);
  }
  else
  {
    if (goal->wait_for_execution)
    {
      res.success = executeProcess(goal);
    }
    else
    {
      boost::thread(&godel_process_execution::KeyenceProcessService::executeProcess, this, goal);
      res.success = true;
    }
  }
}

bool godel_process_execution::KeyenceProcessService::executeProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  ensenso::EnsensoGuard guard;
  // Check for keyence existence
  if (!keyence_client_.exists())
  {
    ROS_ERROR_STREAM("Keyence ROS server is not available on service "
                      << keyence_client_.getService());
    return false;
  }

  // Reset the scans prior to execution
  std_srvs::Trigger dummy_trigger;
  reset_scan_server_.call(dummy_trigger);

  // Prepare the trajectories to run
  godel_msgs::TrajectoryExecution srv_approach;
  srv_approach.request.wait_for_execution = true;
  srv_approach.request.trajectory = goal->trajectory_approach;

  godel_msgs::TrajectoryExecution srv_process;
  srv_process.request.wait_for_execution = true;
  srv_process.request.trajectory = goal->trajectory_process;

  godel_msgs::TrajectoryExecution srv_depart;
  srv_depart.request.wait_for_execution = true;
  srv_depart.request.trajectory = goal->trajectory_depart;

  if (!real_client_.call(srv_approach))
  {
    ROS_ERROR("Execution client unavailable or unable to execute approach trajectory.");
    return false;
  }

  keyence_experimental::ChangeProgram keyence_srv;
  keyence_srv.request.program_no = KEYENCE_PROGRAM_LASER_ON;

  if (!keyence_client_.call(keyence_srv))
  {
    ROS_ERROR_STREAM("Unable to activate keyence (program " << KEYENCE_PROGRAM_LASER_ON << ").");
    return false;
  }

  if (!real_client_.call(srv_process))
  {
    ROS_ERROR("Execution client unavailable or unable to execute process trajectory.");
    return false;
  }

  // Turn keyence off
  keyence_srv.request.program_no = KEYENCE_PROGRAM_LASER_OFF;
  if (!keyence_client_.call(keyence_srv))
  {
    ROS_ERROR_STREAM("Unable to de-activate keyence (program " << KEYENCE_PROGRAM_LASER_OFF
                                                              << ").");
    return false;
  }

  if (!real_client_.call(srv_depart))
  {
    ROS_ERROR("Execution client unavailable or unable to execute departure trajectory.");
    return false;
  }

  return true;
}

bool godel_process_execution::KeyenceProcessService::simulateProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  using industrial_robot_simulator_service::SimulateTrajectory;

  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = goal->trajectory_approach;
  appendTrajectory(aggregate_traj, goal->trajectory_process);
  appendTrajectory(aggregate_traj, goal->trajectory_depart);

  // Pass the trajectory to the simulation service
  SimulateTrajectory srv;
  srv.request.wait_for_execution = goal->wait_for_execution;
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
