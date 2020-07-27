#ifndef GODEL_PROFILE_SCAN_AGGREGATOR_H
#define GODEL_PROFILE_SCAN_AGGREGATOR_H

#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>

namespace godel_scan_server
{

class ProfileScanAggregator
{
public:
  using Profile = pcl::PointCloud<pcl::PointXYZ>;

  ProfileScanAggregator(const std::string& base_frame, const std::string& scan_frame);

  bool addProfile(const pcl::PointCloud<pcl::PointXYZ>& profile);

  void clear();

  const std::vector<Profile>& profiles() const { return profiles_; }
  const std::vector<tf::Transform>& poses() const { return poses_; }

private:
  tf::Transform lookupTransform(const ros::Time& tm) const;

  std::vector<Profile> profiles_;
  std::vector<tf::Transform> poses_;
  tf::TransformListener listener_;
  std::string base_frame_;
  std::string scan_frame_;
};

}

#endif // GODEL_PROFILE_SCAN_AGGREGATOR_H
