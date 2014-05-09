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

#include <godel_surface_detection/scan/robot_scan.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/common.h>
#include <pcl/filters/filter.h>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>
#include <moveit/robot_state/conversions.h>


namespace godel_surface_detection {
namespace scan {


const double RobotScan::PLANNING_TIME = 60.0f;
const double RobotScan::WAIT_MSG_DURATION = 2.0f;
const double RobotScan::MIN_TRAJECTORY_TIME_STEP = 0.5f; // seconds
const double RobotScan::EEF_STEP = 0.04f; // 5cm

RobotScan::RobotScan()
{

	params_.group_name = "manipulator";
	params_.home_position = "home";
	params_.world_frame = "world_frame";
	params_.tcp_frame = "tcp";
	tf::poseTFToMsg(tf::Transform::getIdentity(),params_.tcp_to_cam_pose);
	tf::poseTFToMsg(tf::Transform::getIdentity(),params_.world_to_obj_pose);
	params_.cam_to_obj_zoffset = 0;
	params_.cam_to_obj_xoffset = 0;
	params_.cam_tilt_angle = -M_PI/4;
	params_.sweep_angle_start = 0;
	params_.sweep_angle_end = 2*M_PI;
	params_.scan_topic = "point_cloud";
	params_.scan_target_frame = "world_frame";
	params_.reachable_scan_points_ratio = 0.5f;
	params_.num_scan_points = 20;
	params_.stop_on_planning_error = true;

}

RobotScan::~RobotScan() {
	// TODO Auto-generated destructor stub
}

bool RobotScan::init()
{
	move_group_ptr_ = MoveGroupPtr(new move_group_interface::MoveGroup(params_.group_name));
	move_group_ptr_->setEndEffectorLink(params_.tcp_frame);
	move_group_ptr_->setPoseReferenceFrame(params_.world_frame);
	move_group_ptr_->setPlanningTime(PLANNING_TIME);
	tf_listener_ptr_ = TransformListenerPtr(new tf::TransformListener());
	scan_traj_poses_.clear();
	callback_list_.clear();
	return true;
}

bool RobotScan::load_parameters(std::string ns)
{
	return load_parameters(params_,ns);;
}

bool RobotScan::load_parameters(godel_msgs::RobotScanParameters &params,std::string ns)
{
	ros::NodeHandle nh(ns);

	// pose params
	XmlRpc::XmlRpcValue tcp_to_cam_param, world_to_obj_param;

	// bool params
	bool stop_on_planning_error;

	bool succeeded = nh.getParam("group_name",params.group_name) &&
			nh.getParam("home_position",params.home_position) &&
			nh.getParam("world_frame",params.world_frame) &&
			nh.getParam("tcp_frame",params.tcp_frame) &&
			nh.getParam("tcp_to_cam_pose",tcp_to_cam_param)&&
			nh.getParam("world_to_obj_pose",world_to_obj_param) &&
			nh.getParam("cam_to_obj_zoffset",params.cam_to_obj_zoffset) &&
			nh.getParam("cam_to_obj_xoffset",params.cam_to_obj_xoffset) &&
			nh.getParam("cam_tilt_angle",params.cam_tilt_angle) &&
			nh.getParam("sweep_angle_start",params.sweep_angle_start) &&
			nh.getParam("sweep_angle_end",params.sweep_angle_end) &&
			nh.getParam("scan_topic",params.scan_topic) &&
			nh.getParam("num_scan_points",params.num_scan_points) &&
			nh.getParam("reachable_scan_points_ratio",params.reachable_scan_points_ratio) &&
			nh.getParam("scan_target_frame",params.scan_target_frame) &&
			nh.getParam("stop_on_planning_error",stop_on_planning_error);

	params.stop_on_planning_error = stop_on_planning_error;

	// parsing poses
	succeeded = succeeded && parse_pose_parameter(tcp_to_cam_param,params.tcp_to_cam_pose) &&
			parse_pose_parameter(world_to_obj_param,params.world_to_obj_pose);

	return succeeded;
}

void RobotScan::add_scan_callback(ScanCallback cb)
{
	callback_list_.push_back(cb);
}

bool RobotScan::generate_scan_display_trajectory(moveit_msgs::DisplayTrajectory &traj_data)
{
	// create trajectory
	std::vector<geometry_msgs::Pose> poses;
	bool succeeded = true;
	moveit_msgs::RobotTrajectory robot_traj;
	if(create_scan_trajectory(poses,robot_traj))
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
	succeeded = create_scan_trajectory(poses.poses,robot_traj);
	poses.header.frame_id = params_.world_frame;

	return succeeded;
}

void RobotScan::get_latest_scan_poses(geometry_msgs::PoseArray poses)
{
	poses.poses.insert(poses.poses.begin(),scan_traj_poses_.begin(),scan_traj_poses_.end());
	poses.header.frame_id = params_.world_frame;
}

void RobotScan::publish_scan_poses(std::string topic)
{
	ros::NodeHandle nh;
	ros::Publisher poses_pub = nh.advertise<geometry_msgs::PoseArray>(topic,1,true);
	geometry_msgs::PoseArray poses_msg;
	generate_scan_poses(poses_msg);
	poses_msg.header.frame_id = params_.world_frame;
	poses_pub.publish(poses_msg);
	ros::Duration(1.0f).sleep();
}

bool RobotScan::move_to_pose(geometry_msgs::Pose& target_pose)
{
	move_group_ptr_->setPoseTarget(target_pose,params_.tcp_frame);
	return move_group_ptr_->move();
}

int RobotScan::scan(bool move_only)
{
	// cartesian path generation
	//double alpha_incr = (params_.sweep_angle_end - params_.sweep_angle_start)/(params_.num_scan_points -1);
	double eef_step = EEF_STEP;//1*alpha_incr*params_.cam_to_obj_xoffset;
	double jump_threshold = 0.0f;
	moveit::planning_interface::MoveGroup::Plan path_plan;
	geometry_msgs::PoseArray cartesian_poses;
	path_plan.planning_time_ = PLANNING_TIME;

	// create trajectory
	scan_traj_poses_.clear();
	int poses_reached = 0;
	moveit_msgs::RobotTrajectory robot_traj;
	if(create_scan_trajectory(scan_traj_poses_,robot_traj))
	{
		std::vector<geometry_msgs::Pose> trajectory_poses;

		// inserting all poses
		trajectory_poses.push_back(move_group_ptr_->getCurrentPose(params_.tcp_frame).pose);
		trajectory_poses.insert(trajectory_poses.begin()+1,scan_traj_poses_.begin(),scan_traj_poses_.end());

		for(int i = 1;i < trajectory_poses.size();i++)
		{

			// reset path plan structure
			path_plan.start_state_ = moveit_msgs::RobotState();
			path_plan.trajectory_ = moveit_msgs::RobotTrajectory();

			// filling cartesian poses for next move
			cartesian_poses.poses.clear();
			cartesian_poses.poses.push_back(move_group_ptr_->getCurrentPose(params_.tcp_frame).pose);
			cartesian_poses.poses.push_back(trajectory_poses[i]);

			// getting current robot state
			robot_state::robotStateToRobotStateMsg(*move_group_ptr_->getCurrentState(),
					path_plan.start_state_);

			// creating path plan structure and execute
			if(move_group_ptr_->computeCartesianPath(cartesian_poses.poses,eef_step,jump_threshold,path_plan.trajectory_,true)>=1.0f)
			{
				// applying simple filtering
				apply_simple_trajectory_filter(path_plan.trajectory_);
				//ROS_INFO_STREAM("Robot trajectory "<<i<<":\n"<<path_plan.trajectory_);
			}
			else
			{
				if(params_.stop_on_planning_error)
				{
					ROS_ERROR_STREAM("Path Planning to scan position "<<i <<" failed, quitting scan");
					break;
				}
				else
				{
					ROS_WARN_STREAM("Path Planning to scan position "<<i <<" failed, skipping scan");
					continue;
				}

			}

			if(move_group_ptr_->execute(path_plan))
			{
				poses_reached++;

			}
			else
			{
				if(params_.stop_on_planning_error)
				{
					ROS_ERROR_STREAM("Path Execution to scan position "<<i <<" failed, quitting scan");
					break;
				}
				else
				{
					ROS_WARN_STREAM("Path Execution to scan position "<<i <<" failed, skipping scan");
					continue;
				}
			}

			if(!move_only)
			{
				// get message
				sensor_msgs::PointCloud2ConstPtr msg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(params_.scan_topic,ros::Duration(WAIT_MSG_DURATION));
				pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
				tf::StampedTransform source_to_target_tf;
				if(msg)
				{
					ROS_INFO_STREAM("Cloud message received, converting to target frame '"<< params_.scan_target_frame<<"'");

					// convert to message to point cloud
					pcl::fromROSMsg<pcl::PointXYZ>(*msg,*cloud_ptr);

					// removed nans
					std::vector<int> index;
					pcl::removeNaNFromPointCloud(*cloud_ptr,*cloud_ptr,index);

					// transforming
					if(msg->header.frame_id.compare(params_.scan_target_frame) != 0)
					{
						try
						{
							tf_listener_ptr_->lookupTransform(params_.scan_target_frame,msg->header.frame_id,ros::Time(0),source_to_target_tf);
							pcl_ros::transformPointCloud(*cloud_ptr,*cloud_ptr,source_to_target_tf);
						}
						catch(tf::LookupException &e)
						{
							ROS_ERROR_STREAM("Transform lookup error, using source frame id '"<< msg->header.frame_id<<"'");
						}
						catch(tf::ExtrapolationException &e)
						{
							ROS_ERROR_STREAM("Transform lookup error, using source frame id '"<< msg->header.frame_id<<"'");
						}
					}

					for(std::vector<ScanCallback>::iterator i = callback_list_.begin(); i != callback_list_.end();i++)
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

MoveGroupPtr RobotScan::get_move_group()
{
	return move_group_ptr_;
}

bool RobotScan::create_scan_trajectory(std::vector<geometry_msgs::Pose> &scan_poses,moveit_msgs::RobotTrajectory& scan_traj)
{
	// creating poses
	tf::Transform world_to_tcp = tf::Transform::getIdentity();
	tf::Transform world_to_cam = tf::Transform::getIdentity();
	tf::Transform obj_to_cam_pose = tf::Transform::getIdentity();
	tf::Transform tcp_to_cam_tf,world_to_obj_tf;

	// converting pose msg to tf
	tf::poseMsgToTF(params_.world_to_obj_pose,world_to_obj_tf);
	tf::poseMsgToTF(params_.tcp_to_cam_pose,tcp_to_cam_tf);

	geometry_msgs::Pose pose;
	double alpha;
	double alpha_incr = (params_.sweep_angle_end - params_.sweep_angle_start)/(params_.num_scan_points -1);
	double eef_step = 4*alpha_incr*params_.cam_to_obj_xoffset;
	double jump_threshold = 0.0f;

	// relative transforms
	tf::Transform xoffset_disp = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(params_.cam_to_obj_xoffset,0,0));
	tf::Transform zoffset_disp = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(0,0,params_.cam_to_obj_zoffset));
	tf::Transform rot_alpha_about_z = tf::Transform::getIdentity();
	tf::Transform rot_tilt_about_y = tf::Transform(tf::Quaternion(tf::Vector3(0,1,0),params_.cam_tilt_angle));
	for(int i = 0; i < params_.num_scan_points;i++)
	{
		alpha = params_.sweep_angle_start + alpha_incr * i;
		rot_alpha_about_z = tf::Transform(tf::Quaternion(tf::Vector3(0,0,1),alpha));
		obj_to_cam_pose = zoffset_disp * rot_alpha_about_z*xoffset_disp*rot_tilt_about_y;
		world_to_tcp = world_to_obj_tf * obj_to_cam_pose * tcp_to_cam_tf.inverse();
		tf::poseTFToMsg(world_to_tcp,pose);
		scan_poses.push_back(pose);
	}

	ROS_INFO_STREAM("Computing cartesian path for a trajectory with "<<params_.num_scan_points<<" points, eef_step: "<<eef_step);
	move_group_ptr_->setEndEffectorLink(params_.tcp_frame);

	ROS_INFO_STREAM("Computing cartesian path for link '"<<move_group_ptr_->getEndEffectorLink()<<"'");
	double res = move_group_ptr_->computeCartesianPath(scan_poses,eef_step,jump_threshold,scan_traj,true);
	double success = res >= params_.reachable_scan_points_ratio;
	if(success)
	{
		ROS_INFO_STREAM("Reachable scan poses percentage "<<res<<" is at or above the acceptance threshold of "<<params_.reachable_scan_points_ratio);
	}
	else
	{
		ROS_WARN_STREAM("Reachable scan poses percentage "<<res<<" is below the acceptance threshold of "<<params_.reachable_scan_points_ratio);
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
	q.setRPY(fields_map["rx"],fields_map["ry"],fields_map["rz"]); // fixed axis
	t.setOrigin(pos);
	t.setRotation(q);

	return succeeded;
}

bool RobotScan::parse_pose_parameter(XmlRpc::XmlRpcValue pose_param,geometry_msgs::Pose &pose)
{
	tf::Transform t;
	if(parse_pose_parameter(pose_param,t))
	{
		tf::poseTFToMsg(t,pose);
		return true;
	}
	else
	{
		return false;
	}
}

void RobotScan::apply_simple_trajectory_filter(	moveit_msgs::RobotTrajectory& traj)
{
	std::vector<trajectory_msgs::JointTrajectoryPoint> &points = traj.joint_trajectory.points;

	// filling first and last points with 0 velocities
	points.front().velocities.resize(traj.joint_trajectory.joint_names.size());
	points.back().velocities.resize(traj.joint_trajectory.joint_names.size());

	double dt;
	double dtheta;
	for(unsigned int i = 1;i < points.size()-1; i++)
	{
		trajectory_msgs::JointTrajectoryPoint &p1 = points[i-1];
		trajectory_msgs::JointTrajectoryPoint &p2 = points[i];
		p2.velocities.resize(p2.positions.size());

		// time elapsed between p1 and p2
		dt = (p2.time_from_start - p1.time_from_start).toSec();
		if(dt<MIN_TRAJECTORY_TIME_STEP)
		{
			dt = MIN_TRAJECTORY_TIME_STEP;
			p2.time_from_start = p1.time_from_start + ros::Duration(dt);
		}

		// computing velocity for each joint
		for(unsigned int j = 0; j < p2.velocities.size(); j++)
		{
			p2.velocities[j] = (p2.positions[j] - p1.positions[j])/dt;
/*			ROS_INFO_STREAM("joint velocity for joint "<<traj.joint_trajectory.joint_names[j]<<
					" :" <<p2.velocities[j]<<" for dt: "<<dt);*/
		}
	}

	// checking time stamp for last joint point
	trajectory_msgs::JointTrajectoryPoint &p_last = points.back();
	trajectory_msgs::JointTrajectoryPoint &p_before_last = *(points.end()-2);
	if((p_last.time_from_start - p_before_last.time_from_start).toSec() < MIN_TRAJECTORY_TIME_STEP)
	{
		p_last.time_from_start = p_before_last.time_from_start + ros::Duration(MIN_TRAJECTORY_TIME_STEP);
	}
}

} /* namespace scan */
} /* namespace godel_surface_detection */
