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

template <typename FloatType, typename InputIt>
LineFitSums<FloatType> calculateSums(InputIt begin, InputIt end)
{
  LineFitSums<FloatType> sums;
  sums.x = 0.0;
  sums.y = 0.0;
  sums.x2 = 0.0;
  sums.xy = 0.0;
  sums.n = std::distance(begin, end);

  while (begin != end)
  {
    sums.x += begin->x;
    sums.y += begin->y;
    sums.x2 += ((begin->x) * (begin->x));
    sums.xy += ((begin->x) * (begin->y));

    ++begin;
  }

  return sums;
}

template <typename FloatType>
LineCoef<FloatType> calculateLineCoefs(const LineFitSums<FloatType>& sums)
{
  FloatType x_mean = sums.x / sums.n;
  FloatType y_mean = sums.y / sums.n;

  FloatType slope = (sums.xy - sums.x * y_mean) / (sums.x2 - sums.x * x_mean);
  FloatType intercept = y_mean - slope * x_mean;

  return LineCoef<FloatType>(slope, intercept);
}

template <typename FloatType, typename IOIter>
void adjustWithLineInPlace(const LineCoef<FloatType>& line, IOIter begin, IOIter end)
{
  while (begin != end)
  {
    begin->y -= line(begin->x);
    ++begin;
  }
}

template <typename FloatType, typename IOIter>
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

template <typename FloatType, typename InputIt>
inline FloatType sumSquared(InputIt begin, InputIt end, FloatType init = FloatType())
{
  while (begin != end)
  {
    init += (begin->y) * (begin->y);
    ++begin;
  }
  return init;
}

template <typename FloatType, typename InputIt>
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

template <typename FloatType, typename InputIt> FloatType scoreRms(InputIt begin, InputIt end)
{
  std::size_t n = std::distance(begin, end);
  return std::sqrt(sumSquared<FloatType>(begin, end) / n);
}

template <typename FloatType, typename InputIt> FloatType scoreAvgAbs(InputIt begin, InputIt end)
{
  std::size_t n = std::distance(begin, end);
  return sumAbsValue<FloatType>(begin, end) / n;
}

/////////////////////////////////////////////////////
// Kernel Operations for Applying a Score Function //
/////////////////////////////////////////////////////

template <class Op, typename InputIt, typename OutputIt>
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
