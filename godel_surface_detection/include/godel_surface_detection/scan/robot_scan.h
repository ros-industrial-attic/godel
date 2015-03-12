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

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <boost/function.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/RobotScanParameters.h>

#ifndef ROBOT_SCAN_H_
#define ROBOT_SCAN_H_

namespace godel_surface_detection {
namespace scan {

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

class RobotScan {
public:

	static const double PLANNING_TIME;
	static const double WAIT_MSG_DURATION;
	static const double EEF_STEP;
	static const double MIN_TRAJECTORY_TIME_STEP;
	static const double MIN_JOINT_VELOCITY;

public:
	typedef boost::function<void (pcl::PointCloud<pcl::PointXYZ> &cloud)> ScanCallback;
public:
	RobotScan();
	virtual ~RobotScan();

	bool init();
	bool load_parameters(std::string ns="~");
	static bool load_parameters(godel_msgs::RobotScanParameters &params,std::string ns="");
	void add_scan_callback(ScanCallback cb);
	bool generate_scan_display_trajectory(moveit_msgs::DisplayTrajectory& traj_data);
	bool generate_scan_poses(geometry_msgs::PoseArray& poses);
	void get_latest_scan_poses(geometry_msgs::PoseArray &poses);
	void publish_scan_poses(std::string topic);
	MoveGroupPtr get_move_group();
	bool move_to_pose(geometry_msgs::Pose& target_pose);
	int scan(bool move_only = false);


	static bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,tf::Transform &t);
	static bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,geometry_msgs::Pose &p);

	static void apply_trajectory_parabolic_time_parameterization(robot_trajectory::RobotTrajectory& rt,
			moveit_msgs::RobotTrajectory &traj,unsigned int max_iterations=200,double max_time_change_per_it=.6);

protected:


	// generates circular trajectory above target object
	bool create_scan_trajectory(std::vector<geometry_msgs::Pose> &scan_poses,moveit_msgs::RobotTrajectory& scan_traj);

protected:

	// moveit
	MoveGroupPtr move_group_ptr_;
	TransformListenerPtr tf_listener_ptr_;
	std::vector<geometry_msgs::Pose> scan_traj_poses_;

	// scan
	std::vector<ScanCallback> callback_list_;


public:// parameters

	godel_msgs::RobotScanParameters params_;

/*	// robot move
	std::string group_name_;
	std::string world_frame_;
	std::string tcp_frame_;

	// camera scan poses (It's assumed that the camera is attached to the arm's tcp)
	tf::Transform tcp_to_cam_pose_;
	tf::Transform world_to_obj_pose_;
	double cam_to_obj_zoffset_;
	double cam_to_obj_xoffset_;
	double cam_tilt_angle_; // rotation about relative object's y axis (radians)
	double sweep_angle_start_;
	double sweep_angle_end_;
	int num_scan_points_;
	double reachable_scan_points_ratio_;
	std::string scan_topic_;
	bool stop_on_planning_error_;

	// scan transformation
	std::string scan_target_frame_;*/
};

} /* namespace detection */
} /* namespace godel_surface_detection */
#endif /* ROBOT_SCAN_H_ */
