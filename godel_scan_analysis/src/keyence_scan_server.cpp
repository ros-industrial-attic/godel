#include "godel_scan_analysis/keyence_scan_server.h"

#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

// Constants
const static double TF_WAIT_TIMEOUT = 0.25; // seconds

const static std::string COLOR_CLOUD_TOPIC = "color_cloud";

godel_scan_analysis::ScanServer::ScanServer(const ScanServerConfig& config)
    : map_(new ColorCloud), buffer_(new ColorCloud), config_(config)
{
  ros::NodeHandle nh;
  scan_sub_ = nh.subscribe("profiles", 500, &ScanServer::scanCallback, this);
  cloud_pub_ = nh.advertise<ColorCloud>(COLOR_CLOUD_TOPIC, 1);
  map_->header.frame_id = config_.world_frame;

  // Create publisher for the collected color cloud
  timer_ = nh.createTimer(ros::Duration(config.voxel_grid_publish_period),
                          &ScanServer::publishCloud, this);
}

void godel_scan_analysis::ScanServer::scanCallback(const Cloud& cloud)
{
  // Generate colored point cloud of scan data
  if (!scorer_.analyze(cloud, *buffer_))
    return;

  // Calculate time stamp was processed
  ros::Time stamp;
  stamp.fromNSec(cloud.header.stamp * 1000);

  try
  {
    // Transform scan from optical frame to world frame
    transformScan(*buffer_, stamp);
    // Insert into results cloud
    map_->insert(map_->end(), buffer_->begin(), buffer_->end());
  }
  catch (const tf::TransformException& ex)
  {
    ROS_WARN_STREAM("TF Exception: " << ex.what());
  }

  // Reset buffer
  buffer_->clear();
}

void godel_scan_analysis::ScanServer::publishCloud(const ros::TimerEvent&) const
{
  ColorCloud::Ptr pub_cloud(new ColorCloud);
  // Downsample first
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(map_);
  vg.setLeafSize(config_.voxel_grid_leaf_size, config_.voxel_grid_leaf_size,
                 config_.voxel_grid_leaf_size);
  vg.filter(*pub_cloud);

  cloud_pub_.publish(pub_cloud);
}

void godel_scan_analysis::ScanServer::clear()
{
  map_->clear();
}

void godel_scan_analysis::ScanServer::transformScan(ColorCloud& cloud, const ros::Time& tm) const
{
  tf::StampedTransform transform = findTransform(tm);
  pcl_ros::transformPointCloud(cloud, cloud, transform);
}

inline tf::StampedTransform
godel_scan_analysis::ScanServer::findTransform(const ros::Time& tm) const
{
  tf::StampedTransform transform;
  tf_listener_.waitForTransform(config_.world_frame, config_.scan_frame, tm,
                                ros::Duration(TF_WAIT_TIMEOUT));
  tf_listener_.lookupTransform(config_.world_frame, config_.scan_frame, tm, transform);
  return transform;
}
