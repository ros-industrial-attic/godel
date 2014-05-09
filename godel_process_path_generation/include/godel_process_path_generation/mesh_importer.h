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
 * mesh_importer.h
 *
 *  Created on: May 5, 2014
 *      Author: ros
 */

#ifndef MESH_IMPORTER_H_
#define MESH_IMPORTER_H_

#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>


namespace godel_process_path
{

typedef pcl::PointCloud<pcl::PointXYZINormal> Cloud;
using std::ostream;

/**@brief Represents a point on the boundary of a 2D polygon */
struct PolygonPt
{
  PolygonPt() {};
  PolygonPt(double _x, double _y): x(_x), y(_y) {};
  double x;
  double y;

  inline bool operator==(const PolygonPt& other) const {return x==other.x && y==other.y;}
  inline bool operator!=(const PolygonPt& other) const {return !operator==(other);}

  friend ostream& operator<<(ostream &out, const PolygonPt &ppt);
};
ostream& operator<<(ostream &out, const PolygonPt &ppt)
{
  out << "(" << ppt.x << ", " << ppt.y << ")" << std::endl;
  return out;
}

/**@brief A collection of polygon points that represent the boundary of a polygon */
struct PolygonBoundary
{
  PolygonBoundary() {};
  PolygonBoundary(unsigned int reserve_size) {pts.reserve(reserve_size);}
  std::vector<PolygonPt> pts;

  void reverse() {std::reverse(pts.begin(), pts.end());}

  friend ostream& operator<<(ostream &out, const PolygonBoundary &pb);
};
ostream& operator<<(ostream &out, const PolygonBoundary &pb)
{
  for (std::vector<PolygonPt>::const_iterator pt=pb.pts.begin(), pt_end=pb.pts.end(); pt != pt_end; ++pt)
  {
    out << *pt;
  }
  return out;
}

typedef std::vector<PolygonBoundary> PolygonBoundaryCollection;

class MeshImporter
{
public:
  MeshImporter(): verbose_(false) {};
  virtual ~MeshImporter() {};

  /**@brief Create local coordinate system and boundary data for a point cloud representing a flat surface
   * Note: Boundary data given in local frame of point cloud
   * @param input_mesh PolygonMesh msg containing binary point cloud data and triagonalized mesh data
   * @return true if calculations are successful
   */
  bool calculateBoundaryData(const pcl::PolygonMesh &input_mesh);

  /**@brief Get const reference to the boundary data */
  const PolygonBoundaryCollection& getBoundaries() const
  {
    return boundaries_;
  }


  bool verbose_;        /**< @brief Flag to print additional information */

private:

  void computeLocalPlaneFrame(const Eigen::Hyperplane<double, 3> &plane, const Eigen::Vector3d &centroid);

  /**@brief Compute coefficients of a plane best fit to point cloud
   * Coefficients correspond to ax+by+cz+d=0
   * @param cloud Pointer to pointcloud data
   * @param output Coefficients
   * @return False if plane fit poor, True otherwise.
   */
  static
  bool computePlaneCoefficients(Cloud::ConstPtr cloud, Eigen::Vector4d &output);


  Eigen::Affine3d plane_frame_;                 /**< @brief Transform to local frame of plane */
  PolygonBoundaryCollection boundaries_;        /**< @brief List of boundaries. External boundary must be ordered CCW, internal CW */

};

} /* namespace godel_process_path */
#endif /* MESH_IMPORTER_H_ */
