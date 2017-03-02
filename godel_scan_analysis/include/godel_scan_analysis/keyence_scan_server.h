#ifndef KEYENCE_SCAN_SERVER_H
#define KEYENCE_SCAN_SERVER_H

#include <string>

// scan type
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#include <tf/transform_listener.h>

#include "godel_scan_analysis/scan_roughness_scoring.h"

namespace godel_scan_analysis
{
/**
 * @brief Structure for the configuration parameters associated with
 *        the ScanServer
 */
struct ScanServerConfig
{
  std::string world_frame;
  std::string scan_frame;
  double voxel_grid_leaf_size;
  double voxel_grid_publish_period;
};

/**
 * Defines the ROS interface for a surface-quality-map
 */
class ScanServer
{
public:
  typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

  ScanServer(const ScanServerConfig& config);

  /**
   * Analyzes the passed-in cloud and adds the scored points to the internal map
   */
  void scanCallback(const Cloud& cloud);

  /**
   * A debug call-back to publish point-clouds meant for ROS
   */
  void publishCloud(const ros::TimerEvent&) const;

  /**
   * Queries the underlying map for a colorized point cloud representing the current surface quality
   * of the system
   * @return Shared-Pointer to const PointCloud<PointXYZRGB>
   */
  ColorCloud::ConstPtr getSurfaceQuality() const { return map_; }

  /**
   * @brief Resets accumulated clouds (map_)
   */
  void clear();

private:
  void transformScan(ColorCloud& cloud, const ros::Time& tm) const;
  tf::StampedTransform findTransform(const ros::Time& tm) const;

  RoughnessScorer scorer_; /** Object that scores individual lines */
  ColorCloud::Ptr map_;    /** Data structure that contains colorised surface quality results */
  ColorCloud::Ptr buffer_; /** Temporarily holds scan results for post-processing and tf lookup */
  tf::TransformListener
      tf_listener_;          // for looking up transforms between laser scan and arm position
  ros::Subscriber scan_sub_; // for listening to scans
  ros::Publisher cloud_pub_; // for outputting colored clouds of data
  ros::Timer timer_;         // Publish timer for color cloud
  std::string from_frame_;   // typically laser_scan_frame
  std::string to_frame_;     // typically world_frame
  ScanServerConfig config_;
};

} // end namespace godel_scan_analysis

#endif
