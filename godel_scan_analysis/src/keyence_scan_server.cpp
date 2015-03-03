#include "godel_scan_analysis/keyence_scan_server.h"

#include <pcl_ros/transforms.h>

/*
*/

godel_scan_analysis::ScanServer::ScanServer(const std::string& world_frame, 
                                            const std::string& scan_frame)
  : map_(new ColorCloud)
  , buffer_(new ColorCloud)
  , from_frame_(world_frame)
  , to_frame_(scan_frame)
{
  ros::NodeHandle nh;
  scan_sub_ = nh.subscribe("profiles", 100, &ScanServer::scanCallback, this);
  cloud_pub_ = nh.advertise<ColorCloud>("color_cloud", 1);
  map_->header.frame_id = from_frame_;

  // Create publisher for the collected color cloud
  timer_ = nh.createTimer(ros::Duration(1.0), &ScanServer::publishCloud, this);
}

void godel_scan_analysis::ScanServer::scanCallback(const Cloud& cloud)
{
  // Generate colored point cloud of scan data
  if (!scorer_.analyze(cloud, *buffer_)) return;

  // Calculate time stamp was processed
  ros::Time stamp;
  stamp.fromNSec(cloud.header.stamp * 1000);

  try
  {
    // Transform scan from optical frame to world frame
    transformScan(*buffer_, stamp);    
  }
  catch(...)
  {
    ROS_WARN("No TF");
    return;
  }

  // Insert into results cloud
  map_->insert(map_->end(), buffer_->begin(), buffer_->end());

  // Reset buffer
  buffer_->clear();
}

void godel_scan_analysis::ScanServer::publishCloud(const ros::TimerEvent&) const
{
  cloud_pub_.publish(map_);
}

void godel_scan_analysis::ScanServer::transformScan(ColorCloud& cloud, 
                                                    const ros::Time& tm) const
{
  tf::StampedTransform transform = findTransform(tm);
  pcl_ros::transformPointCloud(cloud, cloud, transform);
}

inline
tf::StampedTransform godel_scan_analysis::ScanServer::findTransform(const ros::Time& tm) const
{
  tf::StampedTransform transform;
  tf_listener_.waitForTransform(from_frame_, to_frame_, tm, ros::Duration(0.25));
  tf_listener_.lookupTransform(from_frame_, to_frame_, tm, transform);
  return transform;
}

