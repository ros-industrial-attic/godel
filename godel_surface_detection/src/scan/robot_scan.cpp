/*
 * robot_scan.cpp
 *
 *  Created on: Apr 14, 2014
 *      Author: ros developer 
 */

#include <godel_surface_detection/scan/robot_scan.h>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>

namespace godel_surface_detection {
namespace scan {

const double RobotScan::PLANNING_TIME = 60.0f;

RobotScan::RobotScan() {
	// TODO Auto-generated constructor stub

}

RobotScan::~RobotScan() {
	// TODO Auto-generated destructor stub
}

bool RobotScan::init()
{
	move_group_ptr_ = MoveGroupPtr(new move_group_interface::MoveGroup(group_name_));
	move_group_ptr_->setEndEffectorLink(tcp_frame_);
	move_group_ptr_->setPoseReferenceFrame(world_frame_);
	move_group_ptr_->setPlanningTime(PLANNING_TIME);
	tf_listener_ptr_ = TransformListenerPtr(new tf::TransformListener());
	scan_traj_points_.clear();
	callback_list_.clear();
	return true;
}

bool RobotScan::load_parameters(std::string ns)
{
	ros::NodeHandle nh(ns);

	// scan transformation
	std::string scan_target_frame_;
	XmlRpc::XmlRpcValue tcp_to_cam_param, world_to_obj_param;
	bool succeeded = nh.getParam("group_name",group_name_) &&
			nh.getParam("world_frame",world_frame_) &&
			nh.getParam("tcp_frame",tcp_frame_) &&
			nh.getParam("tcp_to_cam_pose",tcp_to_cam_param)&&
			nh.getParam("world_to_obj_pose",world_to_obj_param) &&
			nh.getParam("cam_to_obj_zoffset",cam_to_obj_zoffset_) &&
			nh.getParam("cam_to_obj_xoffset",cam_to_obj_xoffset_) &&
			nh.getParam("cam_tilt_angle",cam_tilt_angle_) &&
			nh.getParam("sweep_angle_start",sweep_angle_start_) &&
			nh.getParam("sweep_angle_end",sweep_angle_end_) &&
			nh.getParam("scan_topic",scan_topic_) &&
			nh.getParam("num_scan_points",num_scan_points_);

	// parsing poses
	succeeded = parse_pose_parameter(tcp_to_cam_param,tcp_to_cam_pose_) &&
			parse_pose_parameter(world_to_obj_param,world_to_obj_pose_);

	return succeeded;
}

void RobotScan::add_scan_callback(ScanCallback cb)
{
	callback_list_.push_back(cb);
}

bool RobotScan::scan()
{
	// create trajectory
	scan_traj_points_.clear();
	bool succeeded = true;
	if(create_scan_trajectory(scan_traj_points_))
	{
		for(int i = 0;i < scan_traj_points_.size();i++)
		{
			move_group_ptr_->setPoseTarget(scan_traj_points_[i],tcp_frame_);
			if(move_group_ptr_->move())
			{

			}
		}
	}
	else
	{
		succeeded = false;
	}

	return succeeded;
}

bool RobotScan::create_scan_trajectory(std::vector<geometry_msgs::Pose> &traj)
{
	// creating poses
	tf::Transform world_to_tcp = tf::Transform::getIdentity();
	tf::Transform world_to_cam = tf::Transform::getIdentity();
	tf::Transform obj_to_cam_pose = tf::Transform::getIdentity();
	geometry_msgs::Pose pose;
	double alpha;
	double alpha_incr = (sweep_angle_end_ - sweep_angle_start_)/(num_scan_points_ -1);
	double eef_step = alpha_incr*cam_to_obj_xoffset_;
	double jump_threshold = eef_step;
	for(int i = 0; i < num_scan_points_;i++)
	{
		alpha = sweep_angle_start_ + alpha_incr * i;
		obj_to_cam_pose.setOrigin(tf::Vector3(cam_to_obj_xoffset_,0,cam_to_obj_zoffset_));
		obj_to_cam_pose = tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),alpha) * tf::Quaternion(tf::Vector3(1,0,0),cam_tilt_angle_)) *
				obj_to_cam_pose;
		world_to_tcp = world_to_obj_pose_ * obj_to_cam_pose * tcp_to_cam_pose_.inverse();
		tf::poseTFToMsg(world_to_tcp,pose);
		traj.push_back(pose);
	}

	moveit_msgs::RobotTrajectory robot_traj;
	double res = move_group_ptr_->computeCartesianPath(traj,eef_step,jump_threshold,robot_traj,true);
	double success = res == 1.0d;
	if(success)
	{
		ROS_INFO_STREAM("Points in scan trajectory are "<<res*100.0d<<" reachable");
	}
	else
	{
		ROS_WARN_STREAM("Points in scan trajectory are "<<res*100.0d<<" reachable");
	}

	return success;
}

bool RobotScan::parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,tf::Transform &t)
{
	std::map<std::string,double> fields_map =
			boost::assign::map_list_of("x",0.0d)
			("y",0.0d)
			("z",0.0d)
			("rx",0.0d)
			("ry",0.0d)
			("rz",0.0d);

	// parsing fields
	std::map<std::string,double>::iterator i;
	bool succeeded = true;
	for(i= fields_map.begin();i != fields_map.end();i++)
	{
		if(pose_param.hasMember(i->first) && pose_param[i->first].getType() == XmlRpc::XmlRpcValue::TypeDouble)
		{
			fields_map[i->first] = static_cast<double>(pose_param[i->first]);
		}
		else
		{
			succeeded = false;
			break;
		}
	}

	tf::Vector3 pos = tf::Vector3(fields_map["x"],fields_map["y"],fields_map["z"]);
	tf::Quaternion q;
	q.setEuler(fields_map["rz"],fields_map["ry"],fields_map["rx"]);
	t.setOrigin(pos);
	t.setRotation(q);

	return succeeded;
}

} /* namespace scan */
} /* namespace godel_surface_detection */
