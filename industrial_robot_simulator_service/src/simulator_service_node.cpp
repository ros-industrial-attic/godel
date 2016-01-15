#include <ros/ros.h>

#include <industrial_robot_simulator_service/SimulateTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

// Constants
const static double ACTION_SERVER_WAIT_TIME = 5.0;
const static double DEFAULT_SCALE_FACTOR = 0.2;
const static double GOAL_COMPLETION_BUFFER =
    0.5; // extra time waited past the end of the trajectory

// Utility Functions
static void scaleDurations(trajectory_msgs::JointTrajectory& traj, double scale)
{
  for (std::size_t i = 0; i < traj.points.size(); ++i)
  {
    traj.points[i].time_from_start *= scale;
  }
}

namespace industrial_robot_simulator_service
{
class SimulatorService
{
public:
  SimulatorService(ros::NodeHandle& nh, const std::string& service_name,
                   const std::string& action_name)
      : ac_(action_name, true), service_name_(service_name), scale_factor_(DEFAULT_SCALE_FACTOR)
  {
    execute_trajectory_service_ =
        nh.advertiseService(service_name_, &SimulatorService::simulateTrajectoryCallback, this);

    // Establish connection to robot action server
    if (!ac_.waitForServer(ros::Duration(ACTION_SERVER_WAIT_TIME)))
    {
      std::ostringstream ss;
      ss << "Could not connect to action: '" << action_name << "'";
      ROS_ERROR_STREAM(ss.str());
      throw std::runtime_error(ss.str());
    }
  }

  // Makes call to the simulation action server
  bool
  simulateTrajectoryCallback(industrial_robot_simulator_service::SimulateTrajectory::Request& req,
                             industrial_robot_simulator_service::SimulateTrajectory::Response& res)
  {
    // If empty trajectory, return true right away
    if (req.trajectory.points.empty())
    {
      ROS_WARN("Trajectory simulator recieved empty trajectory");
      return true;
    }

    ROS_INFO_STREAM("Handling new simulation service request");

    // Copy the input header info
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.header = req.trajectory.header;
    goal.trajectory.joint_names = req.trajectory.joint_names;

    // Modify the trajectory such that the first point has a duration of zero.
    // This forces the industrial_robot_simulator to 'snap' the robot to the
    // target position.
    goal.trajectory.points = req.trajectory.points;
    goal.trajectory.points.front().time_from_start = ros::Duration(0.0);

    // Scale the trajectory by the simulator's current time-factor
    scaleDurations(goal.trajectory, scale_factor_);

    ac_.sendGoal(goal);

    // If the user requested that the server block until the simulation is complete, then
    // delay for the time of the trajectory or until the simulator returns. Despite the
    // function returning bool, there isn't really a notion of failure here.
    if (req.wait_for_execution)
    {
      ros::Duration wait_time =
          goal.trajectory.points.back().time_from_start + ros::Duration(GOAL_COMPLETION_BUFFER);
      ROS_DEBUG_STREAM("Waiting for " << wait_time.toSec() << " seconds");
      if (!ac_.waitForResult(wait_time))
      {
        ROS_WARN("Robot Simulator did not successfully complete trajectory");
      }
    }

    return true;
  }

  double scaleFactor() const { return scale_factor_; }

  void setScaleFactor(double scale)
  {
    if (scale > 0.0)
      scale_factor_ = scale;
    else
      ROS_WARN("Cannot set simulator scale factor to number <= 0.0");
  }

private:
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  ros::ServiceServer execute_trajectory_service_;
  std::string service_name_;
  double scale_factor_;
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_service_node");

  ros::NodeHandle nh, pnh("~");

  // Load parameters
  std::string service_name;
  std::string action_name;
  double scale_factor;

  pnh.param<std::string>("service_name", service_name, "simulate_trajectory");
  pnh.param<std::string>("action_name", action_name, "joint_trajectory_action");

  // Initialize the service
  industrial_robot_simulator_service::SimulatorService service(nh, service_name, action_name);

  // Optionally load a new default scale factor
  if (pnh.getParam("scale_factor", scale_factor))
  {
    service.setScaleFactor(scale_factor);
  }

  ROS_INFO_STREAM("Simulation service online");

  ros::spin();
}
