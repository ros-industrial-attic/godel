#include "abb_file_suite/abb_motion_ftp_downloader.h"

#include <fstream>

#include <ros/ros.h>

#include "rapid_generator/rapid_emitter.h"
#include "ftp_upload.h"

// Constants
const static std::string EXECUTE_PROGRAM_SERVICE_NAME = "execute_program";
const static std::string RAPID_MODULE_NAME = "mGodel_blend.mod";

// Utility functions
static double toDegrees(const double radians) { return radians * 180.0 / M_PI; }

static std::vector<double> toDegrees(const std::vector<double>& radians)
{
  std::vector<double> result;
  result.reserve(radians.size());
  for (std::size_t i = 0; i < radians.size(); ++i)
    result.push_back(toDegrees(radians[i]));
  return result;
}

static void linkageAdjust(std::vector<double>& joints) { joints[2] += joints[1]; }

/**
 * Joins a directory and filename while being independent to whether the user
 * added a trailing slash or not.
 *
 * @param  dir      absolute path to directory
 * @param  filename file name
 * @return          absolute path to file
 */
static std::string fileJoin(const std::string& dir, const std::string& filename)
{
  if (dir[dir.length() - 1] == '/')
    return (dir + filename);
  else
    return (dir + std::string("/") + filename);
}

abb_file_suite::AbbMotionFtpDownloader::AbbMotionFtpDownloader(const std::string& ip,
                                                               const std::string& listen_topic,
                                                               ros::NodeHandle& nh,
                                                               const std::string& ftp_user, 
                                                               const std::string& ftp_pass,
                                                               bool j23_coupled,
                                                               const std::string& temp_file_loc)
    : ip_(ip), temp_file_loc_(temp_file_loc), user_(ftp_user), pwd_(ftp_pass), j23_coupled_(j23_coupled)
{
  trajectory_sub_ =
      nh.subscribe(listen_topic, 10, &AbbMotionFtpDownloader::handleJointTrajectory, this);

  server_ = nh.advertiseService(EXECUTE_PROGRAM_SERVICE_NAME,
                                &AbbMotionFtpDownloader::handleServiceCall, this);
}

void abb_file_suite::AbbMotionFtpDownloader::handleJointTrajectory(
    const trajectory_msgs::JointTrajectory& traj)
{
  // Create temporary file
  const std::string temp_file_path = fileJoin(temp_file_loc_, RAPID_MODULE_NAME);
  // generate temporary file with appropriate rapid code
  std::ofstream ofh(temp_file_path.c_str());

  if (!ofh)
  {
    ROS_WARN_STREAM("Could not create file: " << temp_file_path);
    return;
  }

  std::vector<rapid_emitter::TrajectoryPt> pts;
  pts.reserve(traj.points.size());
  for (std::size_t i = 0; i < traj.points.size(); ++i)
  {
    std::vector<double> tmp = toDegrees(traj.points[i].positions);
    if (j23_coupled_)
      linkageAdjust(tmp);

    double duration = 0.0;
    // Timing
    if (i > 0)
    {
      duration = (traj.points[i].time_from_start - traj.points[i - 1].time_from_start).toSec();
    }

    rapid_emitter::TrajectoryPt pt(tmp, duration);
    pts.push_back(pt);
  }

  rapid_emitter::ProcessParams params;
  params.wolf_mode = false;
  rapid_emitter::emitJointTrajectoryFile(ofh, pts, params);
  ofh.flush();
  ofh.close();

  // send to controller
  if (!uploadFile(ip_ + "/PARTMODULES", temp_file_path, user_, pwd_))
  {
    ROS_WARN("Could not upload joint trajectory to remote ftp server");
  }
}

bool abb_file_suite::AbbMotionFtpDownloader::handleServiceCall(
    abb_file_suite::ExecuteProgram::Request& req, abb_file_suite::ExecuteProgram::Response& res)
{
  // Check for existence
  std::ifstream ifh(req.file_path.c_str());
  if (!ifh)
  {
    ROS_WARN("Could not open file '%s'.", req.file_path.c_str());
    return false;
  }
  ifh.close();

  return uploadFile(ip_ + "/PARTMODULES", req.file_path.c_str(),  user_, pwd_);
}
