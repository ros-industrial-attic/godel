/*
	Copyright Apr 15, 2014 Southwest Research Institute

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

static const std::string DISPLAY_TRAJECTORY_TOPIC = "scan_trajectory";
int main(int argc,char** argv)
{
	ros::init(argc,argv,"robot_scan_node");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	ros::NodeHandle nh;

	// publishers
	ros::Publisher traj_pub = nh.advertise<geometry_msgs::PoseArray>(DISPLAY_TRAJECTORY_TOPIC,1,true);

	godel_surface_detection::scan::RobotScan robot_scan;
	if(robot_scan.load_parameters("~/robot_scan") && robot_scan.init())
	{
		geometry_msgs::PoseArray poses_msg;
		robot_scan.get_scan_pose_array(poses_msg);
		ros::Duration loop_duration(0.5f);
		int counter = 0;
		while(ros::ok() && counter < poses_msg.poses.size())
		{
			traj_pub.publish(poses_msg);
			loop_duration.sleep();
			if(robot_scan.move_to_pose(poses_msg.poses[counter]))
			{
				ROS_INFO_STREAM("Move to pose "<<counter<<" succeeded");
				counter++;
			}
			else
			{
				ROS_ERROR_STREAM("Move to pose "<<counter<<" failed");
				break;
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM("Robot scan object did not initialized property");
	}


	return 0;
}




