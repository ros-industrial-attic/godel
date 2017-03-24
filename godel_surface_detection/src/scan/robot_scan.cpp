/*
        Copyright Apr 14, 2014 Southwest Research Institute

        Licensed under the Apache License, Version 2.0 (the "License");
        you may not use this file except in compliance with the License.
        You may obtain a copy of the License at

                http://www.apache.org/licenses/LICENSE-2.0

        Unless required by applicable law or agreed to in writing, software
        distributed under the License is distributed on an "AS IS" BASIS,
        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        See the License for the specific language governing permissions and
        limitations under the License.
*/

#include <scan/robot_scan.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <godel_param_helpers/godel_param_helpers.h>

static const std::string DEFAULT_MOVEIT_PLANNER = "RRTConnectkConfigDefault";

static bool loadPoseParam(ros::NodeHandle& nh, const std::string& name, geometry_msgs::Pose& pose)
{
  using namespace godel_param_helpers;
  return loadParam(nh, name + "/trans/x", pose.position.x) &&
         loadParam(nh, name + "/trans/y", pose.position.y) &&
         loadParam(nh, name + "/trans/z", pose.position.z) &&
         loadParam(nh, name + "/quat/x", pose.orientation.x) &&
         loadParam(nh, name + "/quat/y", pose.orientation.y) &&
         loadParam(nh, name + "/quat/z", pose.orientation.z) &&
         loadParam(nh, name + "/quat/w", pose.orientation.w);
}

namespace godel_surface_detection
{
namespace scan
{

const double RobotScan::PLANNING_TIME = 60.0f;
const double RobotScan::WAIT_MSG_DURATION = 5.0f;
const double RobotScan::MIN_TRAJECTORY_TIME_STEP = 0.8f; // seconds
const double RobotScan::EEF_STEP = 0.05f;                // 5cm
const double RobotScan::MIN_JOINT_VELOCITY = 0.01f;      // rad/sect

RobotScan::RobotScan()
{

  params_.group_name = "manipulator_asus";
  params_.home_position = "home";
  params_.world_frame = "world_frame";
  params_.tcp_frame = "kinect2_move_frame";
  tf::poseTFToMsg(tf::Transform::getIdentity(), params_.tcp_to_cam_pose);
  tf::poseTFToMsg(tf::Transform::getIdentity(), params_.world_to_obj_pose);
  params_.cam_to_obj_zoffset = 0;
  params_.cam_to_obj_xoffset = 0;
  params_.cam_tilt_angle = -M_PI / 4;
  params_.sweep_angle_start = 0;
  params_.sweep_angle_end = 2 * M_PI;
  params_.scan_topic = "point_cloud";
  params_.scan_target_frame = "world_frame";
  params_.reachable_scan_points_ratio = 0.5f;
  params_.num_scan_points = 20;
  params_.stop_on_planning_error = true;
}

bool RobotScan::init()
{
  move_group_ptr_ = MoveGroupPtr(new  moveit::planning_interface::MoveGroupInterface(params_.group_name));
  move_group_ptr_->setEndEffectorLink(params_.tcp_frame);
  move_group_ptr_->setPoseReferenceFrame(params_.world_frame);
  move_group_ptr_->setPlanningTime(PLANNING_TIME);
  move_group_ptr_->setPlannerId(DEFAULT_MOVEIT_PLANNER);
  tf_listener_ptr_ = TransformListenerPtr(new tf::TransformListener());
  scan_traj_poses_.clear();
  callback_list_.clear();
  return true;
}

bool RobotScan::load_parameters(const std::string& filename)
{
  using godel_param_helpers::loadParam;
  using godel_param_helpers::loadBoolParam;

  if (godel_param_helpers::fromFile(filename, params_))
  {
    return true;
  }
  // otherwise load from parameters
  ros::NodeHandle nh("~/robot_scan");
  return loadParam(nh, "group_name", params_.group_name) &&
         loadParam(nh, "home_position", params_.home_position) &&
         loadParam(nh, "world_frame", params_.world_frame) &&
         loadParam(nh, "tcp_frame", params_.tcp_frame) &&
         loadPoseParam(nh, "tcp_to_cam_pose", params_.tcp_to_cam_pose) &&
         loadPoseParam(nh, "world_to_obj_pose", params_.world_to_obj_pose) &&
         loadParam(nh, "cam_to_obj_zoffset", params_.cam_to_obj_zoffset) &&
         loadParam(nh, "cam_to_obj_xoffset", params_.cam_to_obj_xoffset) &&
         loadParam(nh, "cam_tilt_angle", params_.cam_tilt_angle) &&
         loadParam(nh, "sweep_angle_start", params_.sweep_angle_start) &&
         loadParam(nh, "sweep_angle_end", params_.sweep_angle_end) &&
         loadParam(nh, "scan_topic", params_.scan_topic) &&
         loadParam(nh, "num_scan_points", params_.num_scan_points) &&
         loadParam(nh, "reachable_scan_points_ratio", params_.reachable_scan_points_ratio) &&
         loadParam(nh, "scan_target_frame", params_.scan_target_frame) &&
         loadBoolParam(nh, "stop_on_planning_error", params_.stop_on_planning_error);
}

void RobotScan::save_parameters(const std::string& filename)
{
  if (!godel_param_helpers::toFile(filename, params_))
  {
    ROS_WARN_STREAM("Unable to save macro scan-parameters to: " << filename);
  }
}

void RobotScan::add_scan_callback(ScanCallback cb) { callback_list_.push_back(cb); }

bool RobotScan::generate_scan_display_trajectory(moveit_msgs::DisplayTrajectory& traj_data)
{
  // create trajectory
  std::vector<geometry_msgs::Pose> poses;
  bool succeeded = true;
  moveit_msgs::RobotTrajectory robot_traj;
  if (create_scan_trajectory(poses, robot_traj))
  {
    traj_data.trajectory.push_back(robot_traj);
  }
  else
  {
    succeeded = false;
  }
  return succeeded;
}

bool RobotScan::generate_scan_poses(geometry_msgs::PoseArray& poses)
{
  // create trajectory
  bool succeeded = true;
  moveit_msgs::RobotTrajectory robot_traj;
  succeeded = create_scan_trajectory(poses.poses, robot_traj);
  poses.header.frame_id = params_.world_frame;

  return succeeded;
}

void RobotScan::get_latest_scan_poses(geometry_msgs::PoseArray& poses)
{
  poses.poses.insert(poses.poses.begin(), scan_traj_poses_.begin(), scan_traj_poses_.end());
  poses.header.frame_id = params_.world_frame;
}

void RobotScan::publish_scan_poses(std::string topic)
{
  ros::NodeHandle nh;
  ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>(topic, 1, true);
  geometry_msgs::PoseArray poses_msg;
  generate_scan_poses(poses_msg);
  poses_msg.header.frame_id = params_.world_frame;
  poses_pub.publish(poses_msg);
  ros::Duration(1.0f).sleep();
}

bool RobotScan::move_to_pose(geometry_msgs::Pose& target_pose)
{
  move_group_ptr_->setPoseTarget(target_pose, params_.tcp_frame);
  return move_group_ptr_->move();
}

int RobotScan::scan(bool move_only)
{
  // cartesian path generation
  double eef_step = EEF_STEP; // 1*alpha_incr*params_.cam_to_obj_xoffset;
  double jump_threshold = 0.0;
  moveit::planning_interface::MoveGroupInterface::Plan path_plan;
  geometry_msgs::PoseArray cartesian_poses;
  path_plan.planning_time_ = PLANNING_TIME;

  // create trajectory
  scan_traj_poses_.clear();
  int poses_reached = 0;
  moveit_msgs::RobotTrajectory robot_traj;
  if (create_scan_trajectory(scan_traj_poses_, robot_traj))
  {
    std::vector<geometry_msgs::Pose> trajectory_poses;

    // inserting all poses
    trajectory_poses.push_back(move_group_ptr_->getCurrentPose(params_.tcp_frame).pose);
    trajectory_poses.insert(trajectory_poses.begin() + 1, scan_traj_poses_.begin(),
                            scan_traj_poses_.end());

    for (size_t i = 1; i < trajectory_poses.size(); i++)
    {
      // reset path plan structure
      path_plan.start_state_ = moveit_msgs::RobotState();
      path_plan.trajectory_ = moveit_msgs::RobotTrajectory();

      // filling cartesian poses for next move
      cartesian_poses.poses.clear();
      cartesian_poses.poses.push_back(trajectory_poses[i]);

      // creating path plan structure and execute
      move_group_ptr_->setStartStateToCurrentState();

      // Todo: What follows is a hack to get saner motions for the automate demonstration
      // Can fail to plan because the solution is not checked for collisions/limits etc
      // though in practice it works pretty well.
      auto current_state = move_group_ptr_->getCurrentJointValues();
      auto rob_model = move_group_ptr_->getRobotModel();
      moveit::core::RobotState state (rob_model);
      state.setVariablePositions(current_state);
      state.setFromIK(rob_model->getJointModelGroup(params_.group_name), trajectory_poses[i], params_.tcp_frame);
      std::vector<double> to_goto (state.getVariablePositions(), state.getVariablePositions() + current_state.size());
      move_group_ptr_->setJointValueTarget(to_goto);
//      move_group_ptr_->setPoseTarget(trajectory_poses[i], params_.tcp_frame);

      moveit::planning_interface::MoveGroupInterface::Plan my_plan;
      bool success = move_group_ptr_->plan(my_plan);

      if (!success)
      {
        if (params_.stop_on_planning_error)
        {
          ROS_ERROR_STREAM("Path Planning to scan position " << i << " failed, quitting scan");
          break;
        }
        else
        {
          ROS_WARN_STREAM("Path Planning to scan position " << i << " failed, skipping scan");
          continue;
        }
      }

      if (move_group_ptr_->execute(my_plan))
      {
        poses_reached++;
      }
      else
      {
        if (params_.stop_on_planning_error)
        {
          ROS_ERROR_STREAM("Path Execution to scan position " << i << " failed, quitting scan");
          break;
        }
        else
        {
          ROS_WARN_STREAM("Path Execution to scan position " << i << " failed, skipping scan");
          continue;
        }
      }

      if (!move_only)
      {
        // get message
        ros::Duration(1.0).sleep();
        sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(
            params_.scan_topic, ros::Duration(WAIT_MSG_DURATION));
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
        tf::StampedTransform source_to_target_tf;
        if (msg)
        {
          ROS_INFO_STREAM("Cloud message received, converting to target frame '"
                          << params_.scan_target_frame << "'");

          // convert to message to point cloud
          pcl::fromROSMsg<pcl::PointXYZRGB>(*msg, *cloud_ptr);

          // removed nans
          std::vector<int> index;
          pcl::removeNaNFromPointCloud(*cloud_ptr, *cloud_ptr, index);

          // transforming
          if (msg->header.frame_id.compare(params_.scan_target_frame) != 0)
          {
            try
            {
              tf_listener_ptr_->lookupTransform(params_.scan_target_frame, msg->header.frame_id,
                                                ros::Time(0), source_to_target_tf);
              pcl_ros::transformPointCloud(*cloud_ptr, *cloud_ptr, source_to_target_tf);
            }
            catch (tf::LookupException& e)
            {
              ROS_ERROR_STREAM("Transform lookup error, using source frame id '"
                               << msg->header.frame_id << "'");
            }
            catch (tf::ExtrapolationException& e)
            {
              ROS_ERROR_STREAM("Transform lookup error, using source frame id '"
                               << msg->header.frame_id << "'");
            }
          }

          for (std::vector<ScanCallback>::iterator i = callback_list_.begin();
               i != callback_list_.end(); i++)
          {
            (*i)(*cloud_ptr);
          }
        }
        else
        {
          ROS_ERROR_STREAM("Cloud message not received");
        }
      }
      else
      {
        ROS_WARN_STREAM("MOVE_ONLY mode, skipping scan");
      }

      ros::Duration(0.5f).sleep();
    }
  }

  return poses_reached;
}

MoveGroupPtr RobotScan::get_move_group() { return move_group_ptr_; }

bool RobotScan::create_scan_trajectory(std::vector<geometry_msgs::Pose>& scan_poses,
                                       moveit_msgs::RobotTrajectory& scan_traj)
{
  // creating poses
  tf::Transform world_to_tcp = tf::Transform::getIdentity();
  tf::Transform world_to_cam = tf::Transform::getIdentity();
  tf::Transform obj_to_cam_pose = tf::Transform::getIdentity();
  tf::Transform tcp_to_cam_tf, world_to_obj_tf;

  // converting pose msg to tf
  tf::poseMsgToTF(params_.world_to_obj_pose, world_to_obj_tf);
  tf::poseMsgToTF(params_.tcp_to_cam_pose, tcp_to_cam_tf);

  geometry_msgs::Pose pose;
  double alpha;
  double alpha_incr = params_.num_scan_points == 1 ? 0.0 :
      (params_.sweep_angle_end - params_.sweep_angle_start) / (params_.num_scan_points - 1);
  double eef_step = 4 * alpha_incr * params_.cam_to_obj_xoffset;
  double jump_threshold = 0.0f;

  // relative transforms
  tf::Transform xoffset_disp =
      tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(params_.cam_to_obj_xoffset, 0, 0));
  tf::Transform zoffset_disp =
      tf::Transform(tf::Quaternion::getIdentity(), tf::Vector3(0, 0, params_.cam_to_obj_zoffset));
  tf::Transform rot_alpha_about_z = tf::Transform::getIdentity();
  tf::Transform rot_tilt_about_y =
      tf::Transform(tf::Quaternion(tf::Vector3(0, 1, 0), params_.cam_tilt_angle));
  for (int i = 0; i < params_.num_scan_points; i++)
  {
    alpha = params_.sweep_angle_start + alpha_incr * i;
    rot_alpha_about_z = tf::Transform(tf::Quaternion(tf::Vector3(0, 0, 1), alpha));
    obj_to_cam_pose = zoffset_disp * rot_alpha_about_z * xoffset_disp * rot_tilt_about_y;
    world_to_tcp = world_to_obj_tf * obj_to_cam_pose * tcp_to_cam_tf.inverse();
    tf::poseTFToMsg(world_to_tcp, pose);
    scan_poses.push_back(pose);
  }

  move_group_ptr_->setEndEffectorLink(params_.tcp_frame);

  return true;
}

void RobotScan::apply_trajectory_parabolic_time_parameterization(
    robot_trajectory::RobotTrajectory& rt, moveit_msgs::RobotTrajectory& traj,
    unsigned int max_iterations, double max_time_change_per_it)
{

  // applying filter
  trajectory_processing::IterativeParabolicTimeParameterization iter_prmt(max_iterations,
                                                                          max_time_change_per_it);
  iter_prmt.computeTimeStamps(rt);
  rt.getRobotTrajectoryMsg(traj);

  // removing redundant points
  std::vector<trajectory_msgs::JointTrajectoryPoint>& points = traj.joint_trajectory.points;
  std::vector<trajectory_msgs::JointTrajectoryPoint>::iterator iter = points.begin() + 1;
  double dt;
  while (iter != points.end())
  {
    trajectory_msgs::JointTrajectoryPoint& p1 = *(iter - 1);
    trajectory_msgs::JointTrajectoryPoint& p2 = *iter;

    dt = (p2.time_from_start - p1.time_from_start).toSec();
    if (dt < 0.001f)
    {
      iter = points.erase(iter);
    }
    else
    {
      iter++;
    }
  }
}

} /* namespace scan */
} /* namespace godel_surface_detection */
