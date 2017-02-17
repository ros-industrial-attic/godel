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
 *      Author: Dan Solomon
 */

#ifndef MESH_IMPORTER_H_
#define MESH_IMPORTER_H_

#include <ros/ros.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <geometry_msgs/PolygonStamped.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>
#include "godel_process_path_generation/polygon_pts.hpp"
#include "godel_process_path_generation/polygon_utils.h"
#include "godel_process_path_generation/utils.h"

namespace mesh_importer
{

typedef pcl::PointCloud<pcl::PointXYZINormal> Cloud;
typedef godel_process_path::PolygonBoundary PolygonBoundary;
typedef godel_process_path::PolygonBoundaryCollection PolygonBoundaryCollection;

class MeshImporter
{
public:
  MeshImporter(bool verbose = false) : verbose_(verbose){}
  virtual ~MeshImporter(){}

  /**@brief Create local coordinate system and boundary data for a point cloud representing a flat
   * surface
   * Note: Boundary data given in local frame of point cloud
   * @param input_mesh PolygonMesh msg containing binary point cloud data and triagonalized mesh
   * data
   * @return true if calculations are successful
   */
  bool calculateBoundaryData(const pcl::PolygonMesh& input_mesh);

  bool calculateSimpleBoundary(const pcl::PolygonMesh& input_mesh);

  /**@brief Get const reference to the boundary data */
  const PolygonBoundaryCollection& getBoundaries() const { return boundaries_; }

  void getPose(geometry_msgs::Pose& pose) { tf::poseEigenToMsg(plane_frame_, pose); }

  bool verbose_; /**< @brief Flag to print additional information */

private:
  bool applyConcaveHull(const Cloud& plane_cloud, pcl::ModelCoefficients& plane_coeffs,
                        geometry_msgs::PolygonStamped& polygon);

  void computeLocalPlaneFrame(const Eigen::Hyperplane<double, 3>& plane,
                              const Eigen::Vector4d& centroid, const Cloud& cloud);

  /**@brief Compute coefficients of a plane best fit to point cloud
   * Coefficients correspond to ax+by+cz+d=0
   * @param cloud Pointer to pointcloud data
   * @param output Coefficients
   * @return False if plane fit poor, True otherwise.
   */
  bool computePlaneCoefficients(Cloud::ConstPtr cloud, Eigen::Vector4d& output);

  Eigen::Affine3d plane_frame_;          /**< @brief Transform to local frame of plane */
  PolygonBoundaryCollection boundaries_; /**< @brief List of boundaries. External boundary must be
                                            ordered CCW, internal CW */
};

} /* namespace godel_process_path */
#endif /* MESH_IMPORTER_H_ */
