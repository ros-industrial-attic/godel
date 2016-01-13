#ifndef SCAN_ALGORITHMS_H
#define SCAN_ALGORITHMS_H

#include <algorithm>
#include <numeric>

#include "godel_scan_analysis/scan_utilities.h"

namespace rms
{

/////////////////////////////////////////////////////////////////
// Utility functions for fitting and using least-squares lines //
/////////////////////////////////////////////////////////////////

template<typename FloatType, typename InputIt>
LineFitSums<FloatType> calculateSums(InputIt begin, InputIt end)
{
  LineFitSums<FloatType> sums;
  sums.x_ = 0.0;
  sums.y_ = 0.0;
  sums.x2_ = 0.0;
  sums.xy_ = 0.0;
  sums.n_ = std::distance(begin, end);

  while (begin != end)
  {
    sums.x_ += begin->x;
    sums.y_ += begin->y;
    sums.x2_ += ((begin->x) * (begin->x));
    sums.xy_ += ((begin->x) * (begin->y));

    ++begin;
  }

  return sums;
}

template<typename FloatType>
LineCoef<FloatType> calculateLineCoefs(const LineFitSums<FloatType>& sums)
{  
  FloatType x_mean = sums.x_ / sums.n_;
  FloatType y_mean = sums.y_ / sums.n_;

  FloatType slope = (sums.xy_ - sums.x_ * y_mean) / (sums.x2_ - sums.x_ * x_mean);
  FloatType intercept = y_mean - slope * x_mean;

  return LineCoef<FloatType>(slope, intercept); 
}

template<typename FloatType, typename IOIter>
void adjustWithLineInPlace(const LineCoef<FloatType>& line, IOIter begin, IOIter end)
{
  while (begin != end)
  {
    begin->y -= line(begin->x);
    ++begin;
  }
}


template<typename FloatType, typename IOIter>
Scan<FloatType> adjustWithLine(const LineCoef<FloatType>& line, IOIter begin, IOIter end)
{
  Scan<FloatType> scan;
  scan.points.reserve(std::distance(begin, end));
  
  Point<FloatType> temp;

  while (begin != end)
  {
    temp.x = begin->x;
    temp.y = begin->y - line(begin->x);
    scan.points.push_back(temp);
    ++begin;
  }

  return scan;
}

//////////////////////////////////////////////
// Utility functions for scoring algorithms //
//////////////////////////////////////////////

template<typename FloatType, typename InputIt>
inline FloatType sumSquared(InputIt begin, InputIt end, FloatType init = FloatType())
{
  while (begin != end)
  {
    init += (begin->y) * (begin->y);
    ++begin;
  }
  return init;
}

template<typename FloatType, typename InputIt>
inline FloatType sumAbsValue(InputIt begin, InputIt end, FloatType init = FloatType())
{
  while (begin != end)
  {
    init += std::abs(begin->y);
    ++begin;
  }
  return init;
}

//////////////////////////////////////////
// Surface Roughness Scoring Algorithms //
//////////////////////////////////////////

template<typename FloatType, typename InputIt>
FloatType scoreRms(InputIt begin, InputIt end)
{
  std::size_t n = std::distance(begin, end);
  return std::sqrt(sumSquared<FloatType>(begin, end) / n);
}

template<typename FloatType, typename InputIt>
FloatType scoreAvgAbs(InputIt begin, InputIt end)
{
  std::size_t n = std::distance(begin, end);
  return sumAbsValue<FloatType>(begin, end) / n;
}

/////////////////////////////////////////////////////
// Kernel Operations for Applying a Score Function //
/////////////////////////////////////////////////////

template<class Op, typename InputIt, typename OutputIt>
void kernelOp(InputIt wbegin, InputIt wend, InputIt in_end, OutputIt out_begin, Op op = Op()) 
{
  while (wend != in_end)
  {
    *out_begin = op(wbegin, wend);

    ++wend;
    ++wbegin;
    ++out_begin;
  }
}

} // end namespace rms

#endif