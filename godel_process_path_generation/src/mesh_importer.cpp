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
 * mesh_importer.cpp
 *
 *  Created on: May 5, 2014
 *      Author: ros
 */

#include <ros/ros.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/triangle_mesh.h>
#include "godel_process_path_generation/get_boundary.h"
#include "godel_process_path_generation/mesh_importer.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/impl/centroid.hpp>

using Eigen::Vector3d;
using Eigen::Vector4d;

namespace godel_process_path
{

MeshImporter::MeshImporter()
{
}

MeshImporter::~MeshImporter()
{
}

bool MeshImporter::computePlaneCoefficients(const Cloud &cloud, Eigen::Vector4d &output)
{
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients (true);
  // Mandatory
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setDistanceThreshold (0.01);

  seg.setInputCloud (Cloud::Ptr(cloud));
  seg.segment (*inliers, coefficients);
  if (static_cast<double>(inliers->indices.size())/static_cast<double>(cloud.height * cloud.width) < 0.9)
  {
    ROS_WARN("Less than 90% of points included in plane fit.");
    return false;
  }

  ROS_ASSERT(coefficients.values.size() == 4);
  output(0) = coefficients.values.at(0);
  output(1) = coefficients.values.at(1);
  output(2) = coefficients.values.at(2);
  output(3) = coefficients.values.at(3);
  return true;
}

bool MeshImporter::inputData(const pcl::PolygonMesh &input_mesh)
{
  typedef pcl::geometry::PolygonMesh<pcl::geometry::DefaultMeshTraits<> > Mesh;

  Cloud points;
  pcl::fromPCLPointCloud2(input_mesh.cloud,points);


  /* Find plane coefficients from point cloud data.
   * Find centroid of point cloud (projected onto plane) as origin of local coordinate system.
   * Create local coordinate frame for plane
   */
  Eigen::Hyperplane hplane;
  if (!computePlaneCoefficients(points, hplane.coeffs()))
  {
    ROS_WARN("Could not compute plane coefficients");
    return false;
  }

  Eigen::Vector4d centroid4;
  pcl::compute3DCentroid(points, centroid4);
  Eigen::Vector3d centroid = centroid4.head(3);
  centroid = hplane.projection(centroid);       // Project centroid onto plane

  // Check if z_axis (plane normal) is closely aligned with world x_axis:
  // If not, construct transform rotation from X,Z axes. Otherwise, use Y,Z axes.
  const Eigen::Vector3d& z_axis = hplane.coeffs().head(3);
  if (std::abs(z_axis.dot(Vector3d::UnitX())) < 0.8)
  {
    Eigen::Vector3d x_axis = hplane.projection(centroid + Eigen::Vector3d::UnitX());
    plane_frame_.rotation().col(0).head(3) = x_axis.normalized();
    plane_frame_.rotation().col(2).head(3) = z_axis.normalized();
    plane_frame_.rotation().col(1).head(3) = (z_axis.normalized().cross(x_axis.normalized())).normalized();
  }
  else
  {
    Eigen::Vector3d y_axis = hplane.projection(centroid + Eigen::Vector3d::UnitY());
    plane_frame_.rotation().col(1).head(3) = y_axis.normalized();
    plane_frame_.rotation().col(2).head(3) = z_axis.normalized();
    plane_frame_.rotation().col(1).head(3) = (y_axis.normalized().cross(z_axis.normalized())).normalized();
  }
  plane_frame_.translation() = centroid;



  /* Use pcl::geometry::PolygonMesh to extract boundaries.
   * Project boundaries to local plane, and add to boundaries_ list.
   * Note: External boundary is CCW ordered, internal boundaries are CW ordered.
   */
  Mesh mesh;
  // TODO add polygon indices to mesh

  // Extract edges from mesh. Project to plane and add to boundaries_.
  std::vector<Mesh::HalfEdgeIndices> indices;
  pcl_godel::geometry::getBoundBoundaryHalfEdges(mesh, indices);

  //TODO complete function


  return true;
}

} /* namespace godel_process_path */

int
 main (int argc, char** argv)
{
  godel_process_path::MeshImporter mi;
}
