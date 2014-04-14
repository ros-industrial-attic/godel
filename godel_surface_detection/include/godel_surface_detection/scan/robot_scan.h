/*
 * robot_scan.h
 *
 *  Created on: Apr 14, 2014
 *      Author: ros developer 
 */
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <boost/function.hpp>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>

#ifndef ROBOT_SCAN_H_
#define ROBOT_SCAN_H_

namespace godel_surface_detection {
namespace scan {

typedef boost::shared_ptr<move_group_interface::MoveGroup> MoveGroupPtr;
typedef boost::shared_ptr<tf::TransformListener> TransformListenerPtr;

class RobotScan {
public:

	static const double PLANNING_TIME;

public:
	typedef boost::function<bool (pcl::PointCloud<pcl::PointXYZ> &cloud)> ScanCallback;
public:
	RobotScan();
	virtual ~RobotScan();

	bool init();
	bool load_parameters(std::string ns="~");
	void add_scan_callback(ScanCallback cb);
	bool scan();

protected:

	bool parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,tf::Transform &t);

	// generates circular trajectory above target object
	bool create_scan_trajectory(std::vector<geometry_msgs::Pose> &traj);

protected:

	// moveit
	MoveGroupPtr move_group_ptr_;
	TransformListenerPtr tf_listener_ptr_;
	std::vector<geometry_msgs::Pose> scan_traj_points_;

	// scan
	std::vector<ScanCallback> callback_list_;


public:// parameters

	// robot move
	std::string group_name_;
	std::string world_frame_;
	std::string tcp_frame_;

	// camera scan poses (It's assumed that the camera is attached to the arm's tcp)
	tf::Transform tcp_to_cam_pose_;
	tf::Transform world_to_obj_pose_;
	double cam_to_obj_zoffset_;
	double cam_to_obj_xoffset_;
	double cam_tilt_angle_; // rotation relative to object's x axis (radians)
	double sweep_angle_start_;
	double sweep_angle_end_;
	int num_scan_points_;
	std::string scan_topic_;

	// scan transformation
	std::string scan_target_frame_;
};

} /* namespace detection */
} /* namespace godel_surface_detection */
#endif /* ROBOT_SCAN_H_ */
