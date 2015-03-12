#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit/move_group_interface/move_group.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

bool planPathToPosition(moveit::planning_interface::MoveGroup& group, const trajectory_msgs::JointTrajectoryPoint& point);

void executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

void insertCurrentPosition(moveit::planning_interface::MoveGroup& group, trajectory_msgs::JointTrajectory& trajectory);

void rewriteSpeed(const ros::Duration& offset, const ros::Duration& rate, trajectory_msgs::JointTrajectory& trajectory)
{
  ros::Duration from_start = offset;

  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    trajectory.points[i].time_from_start = from_start;

    from_start += rate;
  }
}

bool loadPlan(const std::string& name, trajectory_msgs::JointTrajectory& trajectory)
{
  ROS_INFO_STREAM("Opening file: " << name);

  rosbag::Bag bag;
  bag.open(name, rosbag::bagmode::Read);

  rosbag::View view(bag, rosbag::TopicQuery("trajectory"));

  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
  {
    trajectory_msgs::JointTrajectoryConstPtr traj_ptr = (*it).instantiate<trajectory_msgs::JointTrajectory>();
    
    if (traj_ptr != NULL) 
    {
      trajectory = *traj_ptr;
      return true;
    }
  }
  return false;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trajectory_test_node");
  ros::AsyncSpinner spinner(1); // required for moveit
  spinner.start();


  ros::NodeHandle pnh ("~");

  std::string bagfile_name;
  pnh.param<std::string>("bagfile_name", bagfile_name, "trajectory.bag");

  bool change_speed;
  double speed;
  std::string group_name;
  pnh.param<std::string>("group_name", group_name, "manipulator_tcp");
  pnh.param<bool>("change_speed", change_speed, false);
  pnh.param<double>("speed", speed, 0.0);

  // Configure group
  moveit::planning_interface::MoveGroup group(group_name);
  group.setPlanningTime(10.0);
  group.setPlannerId("RRTstarkConfigDefault");
  
  // Load trajectory for replay
  trajectory_msgs::JointTrajectory traj;
  loadPlan(bagfile_name, traj);

  if (!planPathToPosition(group, traj.points[0]))
  {
    ROS_WARN_STREAM("Could not solve to " << traj.points[0]);
    return -1;
  }

  // Insert current position into trajectory
  insertCurrentPosition(group, traj);


  if (change_speed)
  {
    ROS_INFO_STREAM("CHANGING trajectory speed");
    rewriteSpeed(ros::Duration(2.0), ros::Duration(speed), traj);
  }


  // Execute trajectory
  traj.points.resize(traj.points.size()-1);
  executeTrajectory(traj);

  ROS_INFO_STREAM("Done with trajectory");
}

void executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory)
{
  // Setup action server for trajectory execution
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("joint_trajectory_action", true);
  if (!ac.waitForServer(ros::Duration(2.0)))
  {
    ROS_ERROR_STREAM("Could not connect to action server");
    return;
  }

  control_msgs::FollowJointTrajectoryGoal goal;
  goal.trajectory = trajectory;
  ROS_INFO_STREAM("Sending goal with " << trajectory.points.size() << " points");
  ac.sendGoal(goal);

  if (ac.waitForResult(goal.trajectory.points[goal.trajectory.points.size()-1].time_from_start))
  {
    ROS_INFO_STREAM("Action server reported successful execution");
  } else {
    ROS_WARN_STREAM("Action server could not execute trajectory");
  }
}

bool planPathToPosition(moveit::planning_interface::MoveGroup& group, const trajectory_msgs::JointTrajectoryPoint& point)
{
  group.setJointValueTarget(point.positions);
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  if (success)
  {
    group.execute(my_plan);
    return true;
  }

  return false;
}

void insertCurrentPosition(moveit::planning_interface::MoveGroup& group, trajectory_msgs::JointTrajectory& trajectory)
{
      // get current joint positions
    std::vector<double> group_variable_values;
    group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);

    trajectory_msgs::JointTrajectoryPoint pt;
    pt.positions = group_variable_values;
    std::vector<double> dummy (group_variable_values.size(), 0.0);
    pt.velocities = dummy;
    pt.accelerations = dummy;
    pt.effort = dummy;
    pt.time_from_start = ros::Duration(1.0);
    trajectory.points.insert(trajectory.points.begin(), pt);
}
