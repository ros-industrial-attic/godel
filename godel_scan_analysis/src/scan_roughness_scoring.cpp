#include "godel_scan_analysis/scan_roughness_scoring.h"

#include "godel_scan_analysis/scan_algorithms.h"

#include <ros/ros.h> // debug

#include <math.h> // isfinite

namespace
{
  typedef godel_scan_analysis::RoughnessScorer::Cloud Cloud;
  typedef godel_scan_analysis::RoughnessScorer::ColorCloud ColorCloud;

  // Preprocess clouds
  static rms::Scan<double> filterCloudAndBuildScan(const Cloud& in)
  {
    rms::Scan<double> scan;
    scan.points.reserve(in.points.size());
    for (std::size_t i = 0; i < in.points.size(); ++i)
    {
      rms::Point<double> pt;
      pt.x = in.points[i].x;
      pt.y = in.points[i].z;

      if (std::isfinite(pt.y)) scan.points.push_back(pt);
    }

    return scan;
  }

  static inline double constrainColor(double min, double max, double val)
  {
    return val > max ? max : (val < min ? min : val); 
  }

  // Takes one point and makes a colored pcl point from it
  static pcl::PointXYZRGB makeColoredPoint(const rms::Point<double>& pt, double score)
  {
    static const double max_score = 0.5e-4;
    static const double min_score = 0.0;

    pcl::PointXYZRGB temp;
    temp.x = pt.x;
    temp.y = 0.0;
    temp.z = pt.y;
    temp.r = static_cast<uint8_t>(constrainColor(min_score, max_score, score) / (max_score-min_score) * 255);
    temp.g = 0;
    temp.b = 255 - temp.r;

    return temp;
  }

  // Generates colored points and inserts into out parameter based on scoring
  static void generateColorPoints(const rms::Scan<double>& scan, const rms::Scores& scores, ColorCloud& out)
  {
    long diff = (scan.points.size() - scores.size()) / 2;
    for (size_t i = 0; i < scores.size(); ++i)
    {
      out.points.push_back( makeColoredPoint(scan.points[i + diff], scores[i]) );
    }
  }

  typedef std::vector<rms::Point<double> >::iterator scan_iter;

  static double rmsOp(scan_iter a, scan_iter b)
  {
    return rms::scoreRms<double>(a, b);
  }

  static double absOp(scan_iter a, scan_iter b)
  {
    return rms::scoreAvgAbs<double>(a, b);
  }

} // end anon namespace



godel_scan_analysis::RoughnessScorer::RoughnessScorer()
{}

bool godel_scan_analysis::RoughnessScorer::analyze(const Cloud& in, ColorCloud& out) const
{
  // Preprocess
  rms::Scan<double> scan = filterCloudAndBuildScan(in);
  if (scan.points.size() < 151) return false;

  // Calculate relevant sums/means
  rms::LineFitSums<double> sums = rms::calculateSums<double>(scan.points.begin(), scan.points.end());

  // Fit line
  rms::LineCoef<double> line = rms::calculateLineCoefs(sums);
  rms::Scan<double> adjusted = rms::adjustWithLine(line, scan.points.begin(), scan.points.end());

  // Reserve space for scores
  std:;size_t window = 151;
  std::size_t score_size = std::distance(adjusted.points.begin() + window, adjusted.points.end());
  rms::Scores scores (score_size, 0.0);

  // Apply a surface roughness scoring function
  rms::kernelOp(adjusted.points.begin(), adjusted.points.begin() + window, adjusted.points.end(), scores.begin(), rmsOp);
  
  // Generate output
  generateColorPoints(scan, scores, out);

  // ROS_INFO_STREAM("Min score: " << *std::min(scores.begin(), scores.end()) << " Max: " << *std::max(scores.begin(), scores.end()));

  return true;
}