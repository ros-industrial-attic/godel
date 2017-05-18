#ifndef GODEL_PROFILE_SCAN_FUSION_H
#define GODEL_PROFILE_SCAN_FUSION_H

#include <pcl_ros/point_cloud.h>
#include <tf/tf.h>

namespace godel_scan_analysis
{

struct ProfileFusionParameters
{
  double voxel_leaf_size;
  int min_points_per_voxel;
};

/**
 * @brief The 'fuseProfiles' function merges a collection of profiles taken at very high resolution and frequency
 * at the locations given by 'poses'. This process involves filtering the data prior to fusion, and downsampling
 * the data after combining.
 * @param profiles
 * @param poses
 * @param params
 * @return
 */
pcl::PointCloud<pcl::PointXYZ>::Ptr
fuseProfiles(const std::vector<pcl::PointCloud<pcl::PointXYZ>>& profiles, const std::vector<tf::Transform>& poses,
             const ProfileFusionParameters& params);

}

#endif // GODEL_PROFILE_SCAN_FUSION_H
