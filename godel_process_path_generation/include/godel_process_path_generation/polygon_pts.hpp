/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*
 * polygon_pts.h
 *
 *  Created on: May 10, 2014
 *      Author: Dan Solomon
 */

#ifndef POLYGON_PTS_H_
#define POLYGON_PTS_H_

#include <ostream>
#include <cmath>

using std::ostream;

namespace godel_process_path
{

/**@brief Represents a point on the boundary of a 2D polygon */
struct PolygonPt
{
  PolygonPt() {};
  PolygonPt(double _x, double _y): x(_x), y(_y) {};
  double x;
  double y;

  inline bool operator==(const PolygonPt& other) const {return x==other.x && y==other.y;}
  inline bool operator!=(const PolygonPt& other) const {return !operator==(other);}
  inline PolygonPt operator-(const PolygonPt& other) const {return PolygonPt(x-other.x, y-other.y);}
  inline PolygonPt operator+(const PolygonPt& other) const {return PolygonPt(x+other.x, y+other.y);}
  inline PolygonPt operator*(double d) const {return PolygonPt(x*d, y*d);}
  inline PolygonPt operator/(double d) const {return (*this)*1./d;}

  inline double dist2(const PolygonPt &pt) const {return (pt.x-x)*(pt.x-x) + (pt.y-y)*(pt.y-y);}
  inline double dist(const PolygonPt &pt) const {return std::sqrt(dist2(pt));}
  inline double norm2() const {return x*x + y*y;}
  inline double norm() const {return std::sqrt(norm2());}
  inline double dot(const PolygonPt &pt) {return x*pt.x + y*pt.y;}
  inline double cross(const PolygonPt &pt) {return x*pt.y - y*pt.x;}

  friend ostream& operator<<(ostream &out, const PolygonPt &ppt);
};
ostream& operator<<(ostream &out, const PolygonPt &ppt)
{
  out << "(" << ppt.x << ", " << ppt.y << ")" << std::endl;
  return out;
}

/**@brief A collection of polygon points that represent the boundary of a polygon */
typedef std::vector<PolygonPt> PolygonBoundary;
ostream& operator<<(ostream &out, const PolygonBoundary &pb)
{
  for (PolygonBoundary::const_iterator pt=pb.begin(), pt_end=pb.end(); pt != pt_end; ++pt)
  {
    out << *pt;
  }
  return out;
}

typedef std::vector<PolygonBoundary> PolygonBoundaryCollection;
ostream& operator<<(ostream &out, const PolygonBoundaryCollection &pbc)
{
  size_t idx(0);
  for (PolygonBoundaryCollection::const_iterator pb=pbc.begin(), pb_end=pbc.end(); pb != pb_end; ++pb, ++idx)
  {
    out << "Polygon " << idx << std::endl << *pb;
  }
  return out;
}

std::pair<size_t, float> closestPoint(const PolygonPt &pt, const PolygonBoundary &bnd)
{
  size_t point_num;
  double close_dist2(std::numeric_limits<double>::max());
  for (size_t idx=0, max=bnd.size()-1; idx != max; ++idx )
  {
    double prox2 = pt.dist2(bnd.at(idx));
//    std::cout << "*" << prox2 << std::endl;
    if (prox2 < close_dist2)
    {
      close_dist2 = prox2;
      point_num = idx;
    }
  }
  return std::make_pair(point_num, std::sqrt(static_cast<float>(close_dist2)));
}

} /* namespace godel_process_path */
#endif /* POLYGON_PTS_H_ */
