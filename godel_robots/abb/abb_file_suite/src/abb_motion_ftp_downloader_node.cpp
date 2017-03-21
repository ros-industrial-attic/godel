#include "abb_file_suite/abb_motion_ftp_downloader.h"
#include <ros/ros.h>

// This topic will work with ROS-I standard interfaces such as the action server
// by default
const static std::string DEFAULT_JOINT_LISTENING_TOPIC = "joint_path_command";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "abb_motion_ftp_donwloader");

  ros::NodeHandle nh, pnh("~");

  // load parameters
  std::string ip, topic;
  bool j23_coupled;
  pnh.param<std::string>("controller_ip", ip, "");
  pnh.param<std::string>("topic", topic, DEFAULT_JOINT_LISTENING_TOPIC);

  std::string username, password;
  pnh.param<std::string>("ftp_user", username, "");
  pnh.param<std::string>("ftp_pwd", password, "");

  // Note this lookup is done using a non-private node handle. We assume we are looking up
  // this parameter set in the launch file for many abb drivers.
  nh.param<bool>("J23_coupled", j23_coupled, false);

  abb_file_suite::AbbMotionFtpDownloader ftp(ip, topic, nh, username, password, j23_coupled);

  ROS_INFO("ABB FTP-Trajectory-Downloader (ip %s) initialized. Listening on topic %s.", ip.c_str(),
           topic.c_str());
  ros::spin();

  return 0;
}
