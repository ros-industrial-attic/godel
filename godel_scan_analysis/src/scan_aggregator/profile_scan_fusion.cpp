#include "godel_scan_analysis/profile_scan_fusion.h"
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

pcl::PointCloud<pcl::PointXYZ>::Ptr
godel_scan_analysis::fuseProfiles(const std::vector<pcl::PointCloud<pcl::PointXYZ> > &profiles,
                                  const std::vector<tf::Transform> &poses,
                                  const godel_scan_analysis::ProfileFusionParameters &params)
{
  ROS_ASSERT(profiles.size() == poses.size());

  auto fused_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto final_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  for (std::size_t i = 0; i < profiles.size(); ++i)
  {
    const auto& profile = profiles[i];
    const auto& pose = poses[i];

    // Step 1: Apply filter to individual scan
    pcl::PointCloud<pcl::PointXYZ> buffer = profile; // make a copy to mutate locally

    // Step 2: Transform scan data to base frame
    pcl_ros::transformPointCloud(buffer, buffer, pose);

    // Step 3: Fuse data into map
    fused_cloud->insert(fused_cloud->end(), buffer.begin(), buffer.end());
  }

  // Step 4: Now we have a very dense cloud of data, so let's downsample it for our other items to work on
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(fused_cloud);
  vg.setLeafSize(params.voxel_leaf_size, params.voxel_leaf_size, params.voxel_leaf_size);
  vg.setMinimumPointsNumberPerVoxel(params.min_points_per_voxel);
  vg.filter(*final_cloud);
  // TODO: Other forms of filtering? E.g. minimum samples in a voxel, statistical outlier removal, etc...

  return final_cloud;
}
