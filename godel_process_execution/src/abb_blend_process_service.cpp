#include <godel_process_execution/abb_blend_process_service.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

#include <fstream>

#include "process_utils.h"
#include "rapid_generator/rapid_emitter.h"
#include "abb_file_suite/ExecuteProgram.h"
#include <godel_utils/ensenso_guard.h>

// hack
#include "sensor_msgs/JointState.h"
#include "ros/topic.h"

const static double DEFAULT_JOINT_TOPIC_WAIT_TIME = 5.0; // seconds
const static double DEFAULT_TRAJECTORY_BUFFER_TIME = 5.0; // seconds
const static std::string JOINT_TOPIC_NAME = "/joint_states";

const static std::string THIS_SERVICE_NAME = "blend_process_execution";
const static std::string EXECUTION_SERVICE_NAME = "execute_program";
const static std::string SIMULATION_SERVICE_NAME = "simulate_path";
const static std::string PROCESS_EXE_ACTION_SERVER_NAME = "blend_process_execution_as";

static inline bool compare(const std::vector<double>& a, const std::vector<double>& b,
                           double eps = 0.01)
{
  if (a.size() != b.size())
  {
    ROS_ERROR_STREAM("Can't compare joint vectors of unequal length (" << a.size() << " vs "
                                                                       << b.size() << ")");
    return false;
  }

  double diff = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    diff += std::abs(a[i] - b[i]);
  }

  return diff < eps;
}

static bool waitForExecution(const std::vector<double>& end_goal, const ros::Duration& wait_for,
                             const ros::Duration& time_out)
{
  ensenso::EnsensoGuard guard;
  sensor_msgs::JointStateConstPtr state;
  ros::Time end_time = ros::Time::now() + time_out;

  // wait a fixed amount of time
  wait_for.sleep();

  while (ros::Time::now() < end_time)
  {
    state = ros::topic::waitForMessage<sensor_msgs::JointState>(
        JOINT_TOPIC_NAME, ros::Duration(DEFAULT_JOINT_TOPIC_WAIT_TIME));
    if (!state)
    {
      ROS_WARN("Could not get a joint_state in time");
      return false;
    }
    if (compare(state->position, end_goal))
    {
      ROS_INFO("Goal in tolerance. Returning control.");
      return true;
    }
  }
  return false;
}

static double toDegrees(double rads) { return rads * 180.0 / M_PI; }

static std::vector<double> toDegrees(const std::vector<double>& rads)
{
  std::vector<double> degrees;
  degrees.reserve(rads.size());

  for (std::size_t i = 0; i < rads.size(); ++i)
  {
    degrees.push_back(toDegrees(rads[i]));
  }
  return degrees;
}

static std::vector<rapid_emitter::TrajectoryPt>
toRapidTrajectory(const trajectory_msgs::JointTrajectory& traj, bool j23_coupled)
{
  std::vector<rapid_emitter::TrajectoryPt> rapid_pts;
  rapid_pts.reserve(traj.points.size());

  for (std::size_t i = 0; i < traj.points.size(); ++i)
  {

    // Retrieve and convert joint values to degrees
    std::vector<double> angles = toDegrees(traj.points[i].positions);

    // Account for coupling if necessary
    if (j23_coupled)
    {
      ROS_ASSERT(traj.points[i].positions.size() > 2);
      angles[2] += angles[1];
    }

    // Calculate between point timing
    double duration = 0.0;
    if (i > 0)
    {
      duration = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).toSec();
    }

    // Now we have all the info we need
    rapid_emitter::TrajectoryPt rapid_point(angles, duration);
    rapid_pts.push_back(rapid_point);
  }
  return rapid_pts;
}

static bool writeRapidFile(const std::string& path,
                           const std::vector<rapid_emitter::TrajectoryPt>& traj,
                           unsigned process_start, unsigned process_stop,
                           const rapid_emitter::ProcessParams& params)
{
  std::ofstream fp("/tmp/blend.mod");
  if (!fp)
  {
    ROS_ERROR_STREAM("Unable to create file: " << path);
    return false;
  }

  if (!rapid_emitter::emitRapidFile(fp, traj, process_start, process_stop, params))
  {
    ROS_ERROR("Unable to write to RAPID file for blending process.");
    return false;
  }

  fp.flush();
  return true;
}

godel_process_execution::AbbBlendProcessService::AbbBlendProcessService(ros::NodeHandle& nh) : nh_(nh),
  process_exe_action_server_(nh_,
                           PROCESS_EXE_ACTION_SERVER_NAME,
                           boost::bind(&godel_process_execution::AbbBlendProcessService::executionCallback, this, _1),
                           false)
{
  // Load Robot Specific Parameters
  nh_.param<bool>("J23_coupled", j23_coupled_, false);

  // Create client services
  sim_client_ = nh_.serviceClient<industrial_robot_simulator_service::SimulateTrajectory>(SIMULATION_SERVICE_NAME);
  real_client_ = nh_.serviceClient<abb_file_suite::ExecuteProgram>(EXECUTION_SERVICE_NAME);
  process_exe_action_server_.start();
}

void godel_process_execution::AbbBlendProcessService::executionCallback(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  godel_msgs::ProcessExecutionResult res;
  if (goal->simulate)
  {
    res.success = simulateProcess(goal);
  }
  else
  {
    res.success = executeProcess(goal);
  }
  process_exe_action_server_.setSucceeded(res);
}

bool godel_process_execution::AbbBlendProcessService::executeProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = goal->trajectory_approach;
  appendTrajectory(aggregate_traj, goal->trajectory_process);
  appendTrajectory(aggregate_traj, goal->trajectory_depart);

  // ABB Rapid Emmiter
  std::vector<rapid_emitter::TrajectoryPt> pts = toRapidTrajectory(aggregate_traj, j23_coupled_);

  // RAPID process parameters
  rapid_emitter::ProcessParams params;
  params.spindle_speed = 1.0;
  params.tcp_speed = 200;
  params.wolf_mode = false;
  params.slide_force = 0.0;
  params.output_name = "do_PIO_8";

  // Calculate process start and end indexes
  unsigned start_index = goal->trajectory_approach.points.size();
  unsigned stop_index = start_index + goal->trajectory_process.points.size();

  if (!writeRapidFile("/tmp/blend.mod", pts, start_index, stop_index, params))
  {
    ROS_ERROR("Unable to generate RAPID motion file; Cannot execute process.");
    return false;
  }

  // Call the ABB driver

  abb_file_suite::ExecuteProgram srv;
  srv.request.file_path = "/tmp/blend.mod";

  if (!real_client_.call(srv))
  {
    ROS_ERROR("Unable to upload blending process RAPID module to controller via FTP.");
    return false;
  }

  if (goal->wait_for_execution)
  {
    // If we must wait for execution, then block and listen until robot returns to initial point or times out
    return waitForExecution(goal->trajectory_approach.points.front().positions,
                            aggregate_traj.points.back().time_from_start, // wait for
                            aggregate_traj.points.back().time_from_start +
                                ros::Duration(DEFAULT_TRAJECTORY_BUFFER_TIME)); // timeout
  }
  else
  {
    return true;
  }
}

bool godel_process_execution::AbbBlendProcessService::simulateProcess(
    const godel_msgs::ProcessExecutionGoalConstPtr &goal)
{
  // The simulation server doesn't support any I/O visualizations, so we aggregate the
  // trajectory components and send them all at once
  trajectory_msgs::JointTrajectory aggregate_traj;
  aggregate_traj = goal->trajectory_approach;
  appendTrajectory(aggregate_traj, goal->trajectory_process);
  appendTrajectory(aggregate_traj, goal->trajectory_depart);

  // Pass the trajectory to the simulation service
  industrial_robot_simulator_service::SimulateTrajectory srv;
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
