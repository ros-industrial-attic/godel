/*
 * Software License Agreement (GPLv3 License)
 *
 * Copyright (c) 2014, Southwest Research Institute
 *
 * This file is part of godel. https://github.com/ros-industrial-consortium/godel
 *
 *  godel_polygon_offset is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  godel_polygon_offset is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with godel_polygon_offset.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 * polygon_offset.h
 *
 *  Created on: May 24, 2014
 *      Author: Dan Solomon
 */

#ifndef POLYGON_OFFSET_H_
#define POLYGON_OFFSET_H_

#include <limits>
#include <boost/shared_ptr.hpp>
#include <openvoronoi/voronoidiagram.hpp>
#include <openvoronoi/offset_sorter.hpp>
#include "godel_process_path_generation/polygon_pts.hpp"

using godel_process_path::PolygonBoundaryCollection;

namespace godel_polygon_offset
{

/**@brief PolygonOffset class creates offset paths for closed polygon boundaries.
 * Can handle multiple boundaries of both inside and outside types.
 * offset_/initial_offset_ are used as offset distances.
 * Both line segments and arcs are discretized to line segments.
 * Resulting polygons are sorted for easy machining: Start at innermost and work outwards, then hop
 * to next innermost.
 */
class PolygonOffset
{
public:
  PolygonOffset()
      : verbose_(false), init_ok_(false), offset_(0.), initial_offset_(0.),
        discretization_(std::numeric_limits<double>::max()){};
  virtual ~PolygonOffset(){};

  /**@brief Initialize voronoi diagram with polygon points
   * @param pbc Collection of polygons. CCW(+) ordered points are external boundaries.
   * @return True if voronoi diagram created successfully.
   */
  bool init(const PolygonBoundaryCollection& pbc, double _offset, double _initial_offset,
            double _discretization);

  /**@brief Perform offsets on polygons and arrange in a logical order.
   * Order of polygons is convenient for godel sanding. Smallest path grows outwards, then jumps to
   * next smallest path.
   * @param pbc Resultant polygons.
   * @param offsets List of offset distances corresponding to polygons.
   * @return True if successfully performed offsets.
   */
  bool generateOrderedOffsets(PolygonBoundaryCollection& pbc, std::vector<double>& offsets);

  bool verbose_; /**<Flag to display additional debug messages */

private:
  /**@brief Convert ovd::OffsetLoops (lines/arcs) to PolygonBoundaries (lines)
   * All lines and arcs are discretized (TODO should only arcs be discretized here?)
   * @param polygons Collection of polygons resulting from conversion.
   * @param offset_loops Ordered OffsetLoops by machining order.
   */

  double offset_, initial_offset_; /**<Typical offset and initial offset distance. */
  double discretization_;          /**<Max linear or arc-length distance between adjacent points in
                                      resultant. */

  /*ovd::VoronoiDiagram* vd = new ovd::VoronoiDiagram(1,100); // (r, bins)
   * double r: radius of circle within which all input geometry must fall. use 1 (unit-circle).
   * Scale geometry if necessary.
   * int bins:  bins for face-grid search. roughly sqrt(n), where n is the number of sites is good
   * according to Held. */
  boost::shared_ptr<ovd::VoronoiDiagram> vd_;

  bool init_ok_; /**<Flag to represent that init() completed successfully */
};

} /* namespace godel_polygon_offset */
#endif /* POLYGON_OFFSET_H_ */
