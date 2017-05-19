#include <ros/ros.h>

#include "godel_scan_analysis/profile_scan_fusion.h"
#include "godel_scan_analysis/profile_scan_aggregator.h"
#include <godel_msgs/GetSurfaceScans.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_srvs/Trigger.h>

namespace
{

godel_msgs::LaserScanRecord
createLaserScanRecord(const std::vector<godel_scan_server::ProfileScanAggregator::Profile>& profiles,
                      const std::vector<tf::Transform>& transforms)
{
  godel_msgs::LaserScanRecord msg;

  // Reserve memory
  msg.profiles.resize(profiles.size());
  msg.poses.resize(transforms.size());

  // Convert each cloud
  std::transform(profiles.begin(), profiles.end(), msg.profiles.begin(),
                 [] (const godel_scan_server::ProfileScanAggregator::Profile& profile) {
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(profile, cloud_msg);
    return cloud_msg;
  });

  // Convert each pose
  std::transform(transforms.begin(), transforms.end(), msg.poses.begin(), [] (const tf::Transform& pose) {
    geometry_msgs::Pose pose_msg;
    tf::poseTFToMsg(pose, pose_msg);
    return pose_msg;
  });

  return msg;
}

} // end anonymous namespace

void scanCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& profile,
                  godel_scan_server::ProfileScanAggregator& agg)
{
  agg.addProfile(*profile);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_fusion_test_node");
  ros::NodeHandle nh, pnh ("~");

  // Read params
  std::string base_frame = pnh.param<std::string>("base_frame", "base_link");
  std::string scan_frame = pnh.param<std::string>("scan_frame", "keyence_sensor_optical_frame");

  godel_scan_analysis::ProfileFusionParameters params;
  params.voxel_leaf_size = pnh.param<double>("voxel_size", 0.0005); // 0.5 mm
  params.min_points_per_voxel = pnh.param<int>("min_points_per_voxel", 1);

  // Create aggregator
  godel_scan_server::ProfileScanAggregator aggregator (base_frame, scan_frame);

  // Create services to clear
  ros::ServiceServer reset_service =
      nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("reset_scans",
        [&aggregator] (const std_srvs::TriggerRequest&, std_srvs::TriggerResponse&) {
           aggregator.clear();
           return true;
        }
  );

  // Create subscribers
  ros::Subscriber profile_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("profiles", 0, boost::bind(scanCallback, _1, boost::ref(aggregator)));

  // Create scan pub
  ros::Publisher voxel_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("voxel_grid", 0);

  // Create scan pub trigger
  ros::ServiceServer pub_trigger_service =
      nh.advertiseService<std_srvs::TriggerRequest, std_srvs::TriggerResponse>("pub_voxels",
         [&aggregator, &voxel_pub, params, base_frame] (const std_srvs::TriggerRequest&, std_srvs::TriggerResponse) {
    auto profiles = aggregator.profiles();
    auto poses = aggregator.poses();
    auto cloud = godel_scan_analysis::fuseProfiles(profiles, poses, params);
    cloud->header.frame_id = base_frame;

    voxel_pub.publish(cloud);

    pcl::io::savePCDFile("voxels.pcd", *cloud, true);
    return true;
  });

  ros::ServiceServer get_scan_service =
      nh.advertiseService<godel_msgs::GetSurfaceScansRequest, godel_msgs::GetSurfaceScansResponse>("get_surface_scans",
        [&aggregator, base_frame] (const godel_msgs::GetSurfaceScansRequest&, godel_msgs::GetSurfaceScansResponse& res) {

    res.scans = createLaserScanRecord(aggregator.profiles(), aggregator.poses());
    res.scans.header.frame_id = base_frame;
    res.scans.header.stamp = ros::Time::now();

    return true;
  });

  ros::spin();
  return 0;
}
