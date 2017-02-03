#include <godel_path_execution/path_execution_service.h>
#include <industrial_robot_simulator_service/SimulateTrajectory.h>

const static std::string ACTION_SERVER_NAME = "joint_trajectory_action";
const static double ACTION_EXTRA_WAIT_RATIO = 2.0;   // 20% past end of trajectory
const static double ACTION_SERVICE_WAIT_TIME = 30.0; // seconds
const static char* const ACTION_CONNECTION_FAILED_MSG = "Could not connect to action server.";

const static std::string THIS_SERVICE_NAME = "path_execution";

godel_path_execution::PathExecutionService::PathExecutionService(ros::NodeHandle& nh)
    : ac_(ACTION_SERVER_NAME, true)
{
  server_ = nh.advertiseService<PathExecutionService, godel_msgs::TrajectoryExecution::Request,
                                godel_msgs::TrajectoryExecution::Response>(
      THIS_SERVICE_NAME, &godel_path_execution::PathExecutionService::executionCallback, this);

  // Attempt to connect to the motion action service
  if (!ac_.waitForServer(ros::Duration(ACTION_SERVICE_WAIT_TIME)))
  {
    ROS_ERROR_STREAM(ACTION_CONNECTION_FAILED_MSG);
    throw std::runtime_error(ACTION_CONNECTION_FAILED_MSG);
  }
}

bool godel_path_execution::PathExecutionService::executionCallback(
    godel_msgs::TrajectoryExecution::Request& req, godel_msgs::TrajectoryExecution::Response& res)
{
  // Check preconditions
  if (!ac_.isServerConnected())
  {
    ROS_ERROR_STREAM("Action server is not connected.");
    return false;
  }

  if (req.trajectory.points.empty())
  {
    ROS_WARN_STREAM("Trajectory Execution Service recieved an empty trajectory.");
    return true;
  }

  // Populate goal and send
  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = req.trajectory;
  ac_.sendGoal(goal);

  if (req.wait_for_execution)
  {
    ros::Duration extra_wait =
        goal.trajectory.points.back().time_from_start * ACTION_EXTRA_WAIT_RATIO;
    if (ac_.waitForResult(goal.trajectory.points.back().time_from_start + extra_wait))
    {
      return ac_.getState().state_ == ac_.getState().SUCCEEDED;
    }
    else
    {
      ROS_ERROR_STREAM(__FUNCTION__ << ": Goal failed or did not complete in time.");
      return false;
    }
  }

  return true; // if we don't wait, then always return true immediately
}
