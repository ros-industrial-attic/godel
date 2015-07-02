#ifndef SCAN_UTILITIES_H
#define SCAN_UTILITIES_H

#include <vector>

namespace rms
{

template<typename FloatType>
struct Point
{
  FloatType x,y;
};

template<typename FloatType>
struct LineCoef
{
  FloatType slope_;
  FloatType intercept_;

  LineCoef(double slope, double intercept)
    : slope_(slope)
    , intercept_(intercept)
  {}

  FloatType operator()(const FloatType x) const
  { 
    return slope_ * x + intercept_;
  }
};

template<typename FloatType>
struct LineFitSums
{
  FloatType x_;
  FloatType y_;
  FloatType x2_; // sum of x*x
  FloatType xy_; // sum of x*y
  std::size_t n_;
};

template<typename FloatType>
struct Scan
{
  typedef rms::Point<FloatType> value_type;
  std::vector<value_type> points;
};

typedef std::vector<double> Scores;

} // end namespace rms
#endif                    