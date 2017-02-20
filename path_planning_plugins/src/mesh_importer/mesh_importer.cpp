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
 *      Author: Dan Solomon
 */

#include <ros/ros.h>
#include <boost/bimap.hpp>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/impl/centroid.hpp>
#include <pcl/filters/project_inliers.h>
#include <mesh_importer/mesh_importer.h>
#include "godel_process_path_generation/get_boundary.h"

using Eigen::Vector3d;
using Eigen::Vector4d;

namespace mesh_importer
{

const double CONCAVE_HULL_SIDE_LENGHT = 0.01f;

bool MeshImporter::applyConcaveHull(const Cloud& in, pcl::ModelCoefficients& plane_coeffs,
                                    geometry_msgs::PolygonStamped& polygon)
{
  typedef pcl::PointCloud<pcl::PointXYZ> Cld;

  Cld plane_cloud;
  pcl::copyPointCloud(in, plane_cloud);

  // Project the model inliers
  Cld::Ptr cloud_projected_ptr(new Cld());
  pcl::ModelCoefficients::Ptr coeffs_ptr(new pcl::ModelCoefficients(plane_coeffs));

  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(plane_cloud.makeShared());
  proj.setModelCoefficients(coeffs_ptr);
  proj.filter(*cloud_projected_ptr);

  // Create a Concave Hull representation of the projected inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull(new pcl::PointCloud<pcl::PointXYZ>);
  // pcl::ConcaveHull<pcl::PointXYZ> chull;
  pcl::ConvexHull<pcl::PointXYZ> chull;
  chull.setInputCloud(cloud_projected_ptr);
  // chull.setAlpha (CONCAVE_HULL_SIDE_LENGHT);
  chull.setDimension(2);
  chull.reconstruct(*cloud_hull);

  if (cloud_hull->empty())
  {
    ROS_ERROR_STREAM("Concave hull failed to bound region");
    return false;
  }

  // creating polygon
  geometry_msgs::Point32 p2;
  for (int i = 0; i < cloud_hull->points.size(); i++)
  {

    p2.x = cloud_hull->points[i].x;
    p2.y = cloud_hull->points[i].y;
    p2.z = cloud_hull->points[i].z;

    // polygon.polygon.points.push_back(p1);
    polygon.polygon.points.push_back(p2);
  }

  return true;
}

bool MeshImporter::calculateSimpleBoundary(const pcl::PolygonMesh& input_mesh)
{
  typedef pcl::geometry::TriangleMesh<pcl::geometry::DefaultMeshTraits<> > Mesh;

  plane_frame_.setIdentity();
  boundaries_.clear();

  Cloud::Ptr points(new Cloud);
  pcl::fromPCLPointCloud2(input_mesh.cloud, *points);

  /* Find plane coefficients from point cloud data.
   * Find centroid of point cloud as origin of local coordinate system.
   * Create local coordinate frame for plane.
   */
  Eigen::Hyperplane<double, 3> hplane;
  if (!computePlaneCoefficients(points, hplane.coeffs()))
  {
    ROS_WARN("Could not compute plane coefficients");
    return false;
  }
  if (verbose_)
  {
    ROS_INFO_STREAM("Normal: " << hplane.coeffs().transpose());
  }

  // computes local frame and saves it into the 'plane_frame_' member
  Eigen::Vector4d centroid4;
  pcl::compute3DCentroid(*points, centroid4);
  computeLocalPlaneFrame(hplane, centroid4, *points);
  if (verbose_)
  {
    ROS_INFO_STREAM("Local plane calculated with normal "
                    << hplane.coeffs().transpose() << " and origin " << centroid4.transpose());
  }

  // applying concave hull
  pcl::ModelCoefficients coeffs;
  geometry_msgs::PolygonStamped polygon;
  coeffs.values.clear();
  coeffs.values.push_back(hplane.coeffs()(0));
  coeffs.values.push_back(hplane.coeffs()(1));
  coeffs.values.push_back(hplane.coeffs()(2));
  coeffs.values.push_back(hplane.coeffs()(3));
  applyConcaveHull(*points, coeffs, polygon);

  PolygonBoundary pbound;
  Eigen::Affine3d plane_inverse = plane_frame_.inverse();
  for (int i = 0; i < polygon.polygon.points.size(); i++)
  {
    geometry_msgs::Point32& p = polygon.polygon.points[i];

    Eigen::Vector3d projected_pt = hplane.projection(Eigen::Vector3d(p.x, p.y, p.z));
    Eigen::Vector3d plane_pt = plane_inverse * projected_pt;

    // Check that plane/transform calculations are accurate by testing that transformed points lie
    // on local plane
    if (std::abs(plane_pt(2)) > .001)
    {
      ROS_ERROR_STREAM("z-value of projected/transformed point should be (near) 0 ["
                       << plane_pt.transpose() << "]");
      ROS_ERROR_STREAM("Transform matrix used to project points:\n" << plane_frame_.matrix());
      return false;
    }
    pbound.push_back(godel_process_path::PolygonPt(plane_pt(0), plane_pt(1)));
  }

  boundaries_.push_back(pbound);

  ROS_INFO_STREAM("Added 1 boundary with " << boundaries_[0].size() << " points");

  return true;
}

bool MeshImporter::calculateBoundaryData(const pcl::PolygonMesh& input_mesh)
{
  typedef pcl::geometry::TriangleMesh<pcl::geometry::DefaultMeshTraits<> > Mesh;

  plane_frame_.setIdentity();
  boundaries_.clear();

  Cloud::Ptr points(new Cloud);
  pcl::fromPCLPointCloud2(input_mesh.cloud, *points);

  /* Find plane coefficients from point cloud data.
   * Find centroid of point cloud as origin of local coordinate system.
   * Create local coordinate frame for plane.
   */
  Eigen::Hyperplane<double, 3> hplane;
  if (!computePlaneCoefficients(points, hplane.coeffs()))
  {
    ROS_WARN("Could not compute plane coefficients");
    return false;
  }
  if (verbose_)
  {
    ROS_INFO_STREAM("Normal: " << hplane.coeffs().transpose());
  }

  Eigen::Vector4d centroid4;
  pcl::compute3DCentroid(*points, centroid4);
  computeLocalPlaneFrame(hplane, centroid4, *points);
  if (verbose_)
  {
    ROS_INFO_STREAM("Local plane calculated with normal "
                    << hplane.coeffs().transpose() << " and origin " << centroid4.transpose());
  }

  /* Use pcl::geometry::TriangleMesh to extract boundaries.
   * Project boundaries to local plane, and add to boundaries_ list.
   * Note: External boundary is CCW ordered, internal boundaries are CW ordered.
   */
  Mesh mesh;
  typedef boost::bimap<uint32_t, Mesh::VertexIndex> MeshIndexMap;
  MeshIndexMap mesh_index_map;
  for (size_t ii = 0; ii < input_mesh.polygons.size(); ++ii)
  {
    const std::vector<uint32_t>& vertices = input_mesh.polygons.at(ii).vertices;
    if (vertices.size() != 3)
    {
      ROS_ERROR_STREAM("Found polygon with " << vertices.size()
                                             << " sides, only triangle mesh supported!");
      return false;
    }
    Mesh::VertexIndices vi;
    for (std::vector<uint32_t>::const_iterator vidx = vertices.begin(), viend = vertices.end();
         vidx != viend; ++vidx)
    {
      //      mesh_index_map2.left.count
      if (!mesh_index_map.left.count(*vidx))
      {
        mesh_index_map.insert(MeshIndexMap::value_type(*vidx, mesh.addVertex()));
      }
      vi.push_back(mesh_index_map.left.at(*vidx));
    }
    mesh.addFace(vi.at(0), vi.at(1), vi.at(2));
  }

  // Extract edges from mesh. Project to plane and add to boundaries_.
  std::vector<Mesh::HalfEdgeIndices> boundary_he_indices;
  pcl_godel::geometry::getBoundBoundaryHalfEdges(mesh, boundary_he_indices);

  // For each boundary, project boundary points onto plane and add to boundaries_
  Eigen::Affine3d plane_inverse = plane_frame_.inverse(); // Pre-compute inverse
  for (std::vector<Mesh::HalfEdgeIndices>::const_iterator boundary = boundary_he_indices.begin(),
                                                          b_end = boundary_he_indices.end();
       boundary != b_end; ++boundary)
  {
    PolygonBoundary pbound;
    for (Mesh::HalfEdgeIndices::const_iterator edge = boundary->begin(), edge_end = boundary->end();
         edge != edge_end; ++edge)
    {
      Cloud::PointType cloudpt = points->points.at(
          mesh_index_map.right.at(mesh.getOriginatingVertexIndex(*edge))); // pt on boundary
      Eigen::Vector3d projected_pt = hplane.projection(
          Eigen::Vector3d(cloudpt.x, cloudpt.y, cloudpt.z));   // pt projected onto plane
      Eigen::Vector3d plane_pt = plane_inverse * projected_pt; // pt in plane frame

      // Check that plane/transform calculations are accurate by testing that transformed points lie
      // on local plane
      if (std::abs(plane_pt(2)) > .001)
      {
        ROS_ERROR_STREAM("z-value of projected/transformed point should be (near) 0 ["
                         << plane_pt.transpose() << "]");
        ROS_ERROR_STREAM("Transform matrix used to project points:\n" << plane_frame_.matrix());
        return false;
      }
      pbound.push_back(godel_process_path::PolygonPt(plane_pt(0), plane_pt(1)));
    }

    boundaries_.push_back(pbound);
  }

  return true;
}

void MeshImporter::computeLocalPlaneFrame(const Eigen::Hyperplane<double, 3>& plane,
                                          const Vector4d& centroid, const Cloud& cloud)
{
  Eigen::Vector3d origin = plane.projection(centroid.head<3>()); // Project centroid onto plane
  const Eigen::Vector3d& plane_normal = plane.coeffs().head<3>();
  // Compute major axis of the part
  Eigen::Matrix3d covar_matrix;
  pcl::computeCovarianceMatrixNormalized(cloud, centroid, covar_matrix);
  // Find eigenvectors
  Eigen::Matrix3d evecs; // eigenvectors
  Eigen::Vector3d evals; // eigenvalues
  pcl::eigen33(covar_matrix, evecs, evals);
  // Y-axis can be estimated from the 2nd eigenvector
  Eigen::Vector3d y_axis(evecs(0, 1), evecs(1, 1), evecs(2, 1));
  y_axis = y_axis.normalized();
  // Z-axis computed in plane segmentation step
  Eigen::Vector3d z_axis(plane_normal.normalized());
  // Cross product to obtain X axis
  Eigen::Vector3d x_axis = y_axis.cross(z_axis);

  plane_frame_.matrix().col(0).head<3>() = x_axis;
  plane_frame_.matrix().col(1).head<3>() = y_axis;
  plane_frame_.matrix().col(2).head<3>() = z_axis;

  plane_frame_.translation() = origin;
}

bool MeshImporter::computePlaneCoefficients(Cloud::ConstPtr cloud, Eigen::Vector4d& output)
{
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  // Create the segmentation object
  pcl::SACSegmentation<Cloud::PointType> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(0.01);
  const Cloud::PointType& p0 = cloud->points.at(0);
  Eigen::Vector3f expected_normal(p0.normal_x, p0.normal_y, p0.normal_z);
  seg.setAxis(expected_normal);
  seg.setEpsAngle(0.5);

  seg.setInputCloud(cloud);
  seg.segment(*inliers, coefficients);
  if (coefficients.values.size() != 4)
  {
    ROS_ERROR("Expect 4 coefficients for plane, got %li", coefficients.values.size());
    return false;
  }

  // Check that points match a plane fit
  double fraction_inclusive = 0.9;
  if (static_cast<double>(inliers->indices.size()) /
          static_cast<double>(cloud->height * cloud->width) <
      fraction_inclusive)
  {
    ROS_WARN("Less than 90%% of points included in plane fit.");
    return false;
  }
  ROS_INFO_COND(verbose_, "%f percent of points included in plane fit.",
                static_cast<double>(inliers->indices.size()) /
                    static_cast<double>(cloud->height * cloud->width) * 100.);

  // Check that normal is aligned with pointcloud data
  Eigen::Vector3f actual_normal(coefficients.values.at(0), coefficients.values.at(1),
                                coefficients.values.at(2));
  if (actual_normal.dot(expected_normal) < 0.0)
  {
    ROS_WARN("Flipping RANSAC plane normal");
    actual_normal *= -1;
  }

  output(0) = actual_normal(0);
  output(1) = actual_normal(1);
  output(2) = actual_normal(2);
  output(3) = coefficients.values.at(3);
  return true;
}

} /* namespace godel_process_path */
