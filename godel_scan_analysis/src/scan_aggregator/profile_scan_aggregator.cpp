#include "godel_scan_analysis/profile_scan_aggregator.h"

godel_scan_server::ProfileScanAggregator::ProfileScanAggregator(const std::string &base_frame,
                                                                const std::string &scan_frame)
  : base_frame_(base_frame)
  , scan_frame_(scan_frame)
{
}

bool godel_scan_server::ProfileScanAggregator::addProfile(const pcl::PointCloud<pcl::PointXYZ> &profile)
{
  ros::Time stamp;
  stamp.fromNSec(profile.header.stamp * 1000);

  try
  {
    auto pose = lookupTransform(stamp);
    poses_.push_back(pose);
    profiles_.push_back(profile);
    return true;
  }
  catch (const tf::TransformException& e)
  {
    ROS_WARN_STREAM("TF Exception: " << e.what());
    return false;
  }
}

void godel_scan_server::ProfileScanAggregator::clear()
{
  profiles_.clear();
  poses_.clear();
}

tf::Transform godel_scan_server::ProfileScanAggregator::lookupTransform(const ros::Time &tm) const
{
  const static ros::Duration TF_WAIT_TIMEOUT (0.25); // (seconds)

  tf::StampedTransform transform;
  listener_.waitForTransform(base_frame_, scan_frame_, tm, TF_WAIT_TIMEOUT);
  listener_.lookupTransform(base_frame_, scan_frame_, tm, transform);

  return transform;
}
