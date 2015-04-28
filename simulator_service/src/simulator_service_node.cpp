#include <ros/ros.h>

#include <simulator_service/SimulateTrajectory.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace simulator_service
{
  class SimulatorService
  {
  public:
    SimulatorService(ros::NodeHandle& nh, const std::string& service_name)
      : ac_("joint_trajectory_action", true)
      , service_name_(service_name)
    {
      execute_trajectory_service_ = nh.advertiseService(service_name_, 
        &SimulatorService::simulateTrajectoryCallback, 
        this);

      // Establish connection to robot action server
      if (!ac_.waitForServer(ros::Duration(5.0)))
      {
        ROS_ERROR_STREAM("Could not connect to action server");
        throw std::runtime_error("Could not connect to joint_trajectory_action server");
      }

    }

    // Makes call to the simulation action server
    bool simulateTrajectoryCallback(simulator_service::SimulateTrajectory::Request& req,
                                    simulator_service::SimulateTrajectory::Response& res)
    {
      if (req.trajectory.points.empty())
      {
        ROS_WARN_STREAM("Can not simulate empty trajectory");
        return false;
      }

      ROS_INFO_STREAM("Handling new simulation service request");
      control_msgs::FollowJointTrajectoryGoal goal;
      

      // First set the simulator to the initial position of the sim
      goal.trajectory = req.trajectory;
      goal.trajectory.points.clear();
      goal.trajectory.points.push_back(req.trajectory.points.front());
      goal.trajectory.points.front().time_from_start = ros::Duration(0.0);
      ac_.sendGoal(goal);
      ac_.waitForResult(ros::Duration(0.5));
      // Then send the whole trajectory
      goal.trajectory = req.trajectory;
      ac_.sendGoal(goal);

      if (req.wait_for_execution && !goal.trajectory.points.empty())
      {
        ros::Duration wait_time = goal.trajectory.points.back().time_from_start;
        ac_.waitForResult(wait_time);
      }

      return true;
    }

  private:
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
    ros::ServiceServer execute_trajectory_service_;
    std::string service_name_;
  };
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simulation_service_node");

  ros::NodeHandle nh;
  simulator_service::SimulatorService service(nh, "simulate");
  
  ROS_INFO_STREAM("Simulation service online");

  ros::spin();
}