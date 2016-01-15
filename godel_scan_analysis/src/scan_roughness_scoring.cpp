#include "godel_scan_analysis/scan_roughness_scoring.h"

#include "godel_scan_analysis/scan_algorithms.h"

#include <ros/ros.h> // debug

#include <math.h> // isfinite

/*
  I have a variety of ideas for making this code faster if it was ever needed:
  1) Keep scan data as an array of x and array of y for better cache coherency on windowing
  2) The windowing could be made to operate in O(n) - touching every point only once
  3) Pre-calculate x values (which are known)
  4) Pre-calculate colors
  5) Perhaps return an index map like PCL does from the filter function so
     we don't need to make a copy of the scan
  6) Pre-allocate or use stack space (std::array?) for storage space
*/

const static double DEFAULT_MAX_SCORE =
    0.00003; // RMS value about a point that is considered out of spec
const static double DEFAULT_MIN_SCORE = 0.0; // Min RMS value; Used for coloring together with above
const static unsigned WINDOW_SIZE =
    30; // The # of points in a window; equivalent to 0.05 mm * WINDOW_SIZE for Keyence

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

    if (std::isfinite(pt.y))
      scan.points.push_back(pt);
  }

  return scan;
}

static inline double constrainValue(double min, double max, double val)
{
  return (val > max) ? max : ((val < min) ? min : val);
}

// Takes one point and makes a colored pcl point from it
static pcl::PointXYZRGB makeColoredPoint(const rms::Point<double>& pt, double score)
{
  // TODO: put these colorization values into the params struct
  static const double max_score = DEFAULT_MAX_SCORE;
  static const double min_score = DEFAULT_MIN_SCORE;

  pcl::PointXYZRGB temp;
  temp.x = pt.x;
  temp.y = 0.0;
  temp.z = pt.y;
  temp.r = static_cast<uint8_t>(constrainValue(min_score, max_score, score) /
                                (max_score - min_score) * 255);
  temp.g = 0;
  temp.b = 255 - temp.r;

  return temp;
}

// Generates colored points and inserts into out parameter based on scoring
static void generateColorPoints(const rms::Scan<double>& scan, const rms::Scores& scores,
                                ColorCloud& out)
{
  long diff = (scan.points.size() - scores.size()) / 2;
  for (size_t i = 0; i < scores.size(); ++i)
  {
    out.points.push_back(makeColoredPoint(scan.points[i + diff], scores[i]));
  }
}

typedef std::vector<rms::Point<double> >::iterator scan_iter;

static double rmsOp(scan_iter a, scan_iter b) { return rms::scoreRms<double>(a, b); }

static double absOp(scan_iter a, scan_iter b) { return rms::scoreAvgAbs<double>(a, b); }

static double localLine(scan_iter a, scan_iter b)
{
  // Calculate relevant sums/means
  rms::LineFitSums<double> sums = rms::calculateSums<double>(a, b);

  // Fit line
  rms::LineCoef<double> line = rms::calculateLineCoefs(sums);

  rms::Scan<double> adjusted = rms::adjustWithLine(line, a, b);

  return rms::scoreRms<double>(adjusted.points.begin(), adjusted.points.end());
}

} // end anon namespace

godel_scan_analysis::RoughnessScorer::RoughnessScorer() {}

bool godel_scan_analysis::RoughnessScorer::analyze(const Cloud& in, ColorCloud& out) const
{
  // Preprocess
  rms::Scan<double> scan = filterCloudAndBuildScan(in);
  if (scan.points.size() < WINDOW_SIZE)
    return false;

  // Calculate relevant sums/means
  rms::LineFitSums<double> sums =
      rms::calculateSums<double>(scan.points.begin(), scan.points.end());

  // Fit line
  rms::LineCoef<double> line = rms::calculateLineCoefs(sums);
  rms::Scan<double> adjusted = rms::adjustWithLine(line, scan.points.begin(), scan.points.end());

  // Reserve space for scores
  std::size_t score_size =
      std::distance(adjusted.points.begin() + WINDOW_SIZE, adjusted.points.end());
  rms::Scores scores(score_size, 0.0);

  // Apply a surface roughness scoring function
  rms::kernelOp(adjusted.points.begin(), adjusted.points.begin() + WINDOW_SIZE,
                adjusted.points.end(), scores.begin(), localLine);

  // Generate output
  generateColorPoints(scan, scores, out);

  return true;
}
