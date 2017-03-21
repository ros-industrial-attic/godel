#ifndef ABB_MOTION_FTP_DOWNLOADER_H
#define ABB_MOTION_FTP_DOWNLOADER_H

#include <ros/subscriber.h>
#include <ros/service_server.h>
#include <trajectory_msgs/JointTrajectory.h>
#include "abb_file_suite/ExecuteProgram.h"

namespace abb_file_suite
{

/**
 * @brief The class provides an alternative driver interface to ABB robots
 *        running an FTP server. It offers a joint trajectory interface and
 *        a service that can execute arbitrary RAPID scripts.
 */
class AbbMotionFtpDownloader
{
public:
  AbbMotionFtpDownloader(const std::string& ip, const std::string& listen_topic,
                         ros::NodeHandle& nh,
                         const std::string& ftp_user, const std::string& ftp_pass,
                         bool j23_coupled = false,
                         const std::string& temp_file_loc = std::string("/tmp"));

  /**
   * Callback handler that executes a new joint trajectory. This is accomplished
   * by writing a RAPID file that encodes the requested joint motions and timing.
   * This file is then uploaded via FTP
   */
  void handleJointTrajectory(const trajectory_msgs::JointTrajectory& traj);

  /**
   * This service call handler will attempt to directly upload a RAPID file
   * to the robot from the path given in the request.
   * @param  req Contains the ABSOLUTE path the RAPID file to upload
   * @param  res No fields in the return value
   * @return     True if the FTP transfer was completed; note this doesn't mean the robot
   *             succeeded or even read the complete file.
   */
  bool handleServiceCall(abb_file_suite::ExecuteProgram::Request& req,
                         abb_file_suite::ExecuteProgram::Response& res);

private:
  ros::Subscriber trajectory_sub_;
  ros::ServiceServer server_;
  const std::string ip_;
  const std::string temp_file_loc_;
  const std::string user_;
  const std::string pwd_;
  bool j23_coupled_; /** joints 2 and 3 are coupled (as in ABB IRB2400) */
};
}

#endif
