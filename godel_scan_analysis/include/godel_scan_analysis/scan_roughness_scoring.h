#ifndef SCAN_ROUGHNESS_SCORING_H
#define SCAN_ROUGHNESS_SCORING_H

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

namespace godel_scan_analysis
{
// TODO: add scoring params to this struct
struct ScoringParams
{
};

/**
 * This class, once setup, scores individual laser scans
 * with a given algorithm, colorizes them, and pushes them
 * back out to a colorized cloud.
 */
class RoughnessScorer
{
public:
  // Input
  typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
  // output
  typedef pcl::PointCloud<pcl::PointXYZRGB> ColorCloud;

  RoughnessScorer();

  bool analyze(const Cloud& in, ColorCloud& out) const;

private:
  ScoringParams params_;
};
}

#endif
