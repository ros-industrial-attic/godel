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

#include <pcl/PolygonMesh.h>
#include <Eigen/Geometry>


namespace godel_process_path
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;

struct PolygonPt
{
  PolygonPt() {};
  PolygonPt(double _x, double _y)
  {
    x = _x;
    y = _y;
  }
  double x;
  double y;
};

struct PolygonBoundary
{
  PolygonBoundary() {};
  PolygonBoundary(unsigned int reserve_size)
  {
    pts.reserve(reserve_size);
  }
  std::vector<PolygonPt> pts;
};

class MeshImporter
{
public:
  MeshImporter();
  virtual ~MeshImporter();
  bool inputData(const pcl::PolygonMesh &input_mesh);

private:
  bool computePlaneCoefficients(Cloud::ConstPtr cloud, Eigen::Vector4d &output);


  Eigen::Affine3d plane_frame_;                 /**< @brief Transform to local frame of plane **/
  std::list<PolygonBoundary> boundaries_;       /**< @brief List of boundaries. External boundary must be ordered CCW, internal CW **/


};

} /* namespace godel_process_path */
#endif /* MESH_IMPORTER_H_ */
