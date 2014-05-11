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
 *      Author: ros
 */

#ifndef POLYGON_PTS_H_
#define POLYGON_PTS_H_

#include <ostream>

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

  inline double dist2(const PolygonPt &pt) const {return (pt.x-x)*(pt.x-x) + (pt.y-y)*(pt.y-y);}
  inline double dist(const PolygonPt &pt) const {return std::sqrt(dist2(pt));}

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
  for (std::vector<PolygonPt>::const_iterator pt=pb.begin(), pt_end=pb.end(); pt != pt_end; ++pt)
  {
    out << *pt;
  }
  return out;
}

typedef std::vector<PolygonBoundary> PolygonBoundaryCollection;

std::pair<PolygonBoundary::iterator, float> closestPoint(const PolygonPt &pt, PolygonBoundary &bnd)
{
  PolygonBoundary::iterator close_pt;
  double dist2(std::numeric_limits<double>::max());
  for (PolygonBoundary::iterator bnd_pt=bnd.begin(), bnd_end=bnd.end(); bnd_pt!=bnd_end; ++bnd_pt)
  {
    if (pt.dist2(*bnd_pt) < dist2)
    {
      dist2 = pt.dist2(*bnd_pt);
      close_pt = bnd_pt;
    }
  }
  return std::make_pair(close_pt, std::sqrt(static_cast<float>(dist2)));
}

} /* namespace godel_process_path */
#endif /* POLYGON_PTS_H_ */
