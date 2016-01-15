#ifndef SCAN_UTILITIES_H
#define SCAN_UTILITIES_H

#include <vector>

namespace rms
{

template <typename FloatType> struct Point
{
  FloatType x, y;
};

template <typename FloatType> struct LineCoef
{
  FloatType slope;
  FloatType intercept;

  LineCoef(double slope, double intercept) : slope(slope), intercept(intercept) {}

  FloatType operator()(const FloatType x) const { return slope * x + intercept; }
};

template <typename FloatType> struct LineFitSums
{
  FloatType x;
  FloatType y;
  FloatType x2; // sum of x*x
  FloatType xy; // sum of x*y
  std::size_t n;
};

template <typename FloatType> struct Scan
{
  typedef rms::Point<FloatType> value_type;
  std::vector<value_type> points;
};

typedef std::vector<double> Scores;

} // end namespace rms
#endif
