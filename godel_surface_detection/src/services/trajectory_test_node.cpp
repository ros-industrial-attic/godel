#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#include <moveit/move_group_interface/move_group.h>

#include <eigen_conversions/eigen_msg.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>


bool planPathToPosition(moveit::planning_interface::MoveGroup& group,
                        const trajectory_msgs::JointTrajectoryPoint& point);

void executeTrajectory(const trajectory_msgs::JointTrajectory& trajectory);

void insertCurrentPosition(moveit::planning_interface::MoveGroup& group, trajectory_msgs::JointTrajectory& trajectory);

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

  // Read parameters to the server
  std::string bagfile_name;
  pnh.param<std::string>("bagfile_name", bagfile_name, "trajectory.bag");

  std::string group_name;
  pnh.param<std::string>("group_name", group_name, "manipulator");

  double planning_time;
  pnh.param<double>("planning_time", planning_time, 10.0);

  // Configure group
  moveit::planning_interface::MoveGroup group(group_name);
  group.setPlanningTime(planning_time);
  group.setPlannerId("RRTstarkConfigDefault");
  group.setPoseReferenceFrame("world_frame");

  const std::vector<std::string>& names = group.getJoints();

  // Load trajectory for replay
  trajectory_msgs::JointTrajectory traj;
  loadPlan(bagfile_name, traj);

  if (!planPathToPosition(group, traj.points[0]))
  {
    ROS_WARN_STREAM("Couldn't make joint move to start pos");
    return -1;
  }

  // Insert current position into trajectory
  insertCurrentPosition(group, traj);

  // Execute trajectory
  // Beware: if the trajectory starts and stops at the same point, the ROS-I
  // controllers will often just ignore your input. One solution, however bad,
  // is to truncate a point or two from the trajectory using resize.
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

  ros::Duration wait_time = goal.trajectory.points.back().time_from_start + ros::Duration(2.0);
  if (ac.waitForResult(wait_time))
  {
    ROS_INFO_STREAM("Action server reported successful execution");
  } else {
    ROS_WARN_STREAM("Action server could not execute trajectory");
  }
}

// Joint motion
bool planPathToPosition(moveit::planning_interface::MoveGroup& group,
                        const trajectory_msgs::JointTrajectoryPoint& point)
{
  moveit::planning_interface::MoveGroup::Plan my_plan;

  group.setJointValueTarget(point.positions);
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
