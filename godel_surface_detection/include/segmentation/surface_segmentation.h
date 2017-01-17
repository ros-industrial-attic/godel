#ifndef SURFACE_SEGMENTATION_H
#define SURFACE_SEGMENTATION_H

#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/geometry/mesh_base.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/geometry/quad_mesh.h>
#include <pcl/geometry/polygon_mesh.h>
#include <pcl/geometry/mesh_conversion.h>
#include <pcl/features/boundary.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/filter.h>
#include <Eigen/Geometry>
#include <Eigen/StdVector>

static const int MAX_CLUSTER_SIZE = 100000;
static const int MIN_CLUSTER_SIZE = 2500;
static const int NUM_NEIGHBORS = 30;


template <bool IsManifoldT>
struct MeshTraits
{
  typedef pcl::PointXYZRGBNormal                       VertexData;
  typedef pcl::geometry::NoData                        HalfEdgeData;
  typedef pcl::geometry::NoData                        EdgeData;
  typedef pcl::geometry::NoData                        FaceData;
  typedef boost::integral_constant <bool, IsManifoldT> IsManifold;
};

typedef MeshTraits <true >                               ManifoldMeshTraits;
typedef pcl::geometry::PolygonMesh <ManifoldMeshTraits>  Mesh;
typedef typename Mesh::HalfEdgeIndex                     HalfEdgeIndex;
typedef typename Mesh::HalfEdgeIndices                   HalfEdgeIndices;
typedef typename Mesh::InnerHalfEdgeAroundFaceCirculator IHEAFC;


/** @class world_background_subtraction
@brief Maintains record of baseline sensor data to provide method to remove them leaving only new objects in the scene
*/
class SurfaceSegmentation
{
public:

  // mesh results
  pcl::PolygonMesh triangles_;
  std::vector<int> parts_;
  std::vector<int> states_;

  // segmentation results
  std::vector <pcl::PointIndices> clusters_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr input_cloud_;
  Mesh HEM_;

  // smoothing filter
  int num_coef_;
  std::vector<double> coef_;
  double gain_;

  // search terms
  double radius_;

  // Constructors/Destructors
  SurfaceSegmentation();
  ~SurfaceSegmentation();
  SurfaceSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);

  // Clouds
  void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);
  void addCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr icloud);
  void getBoundaryCloud(pcl::PointCloud<pcl::Boundary>::Ptr &boundary_cloud);
  void getSurfaceClouds(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &surface_clouds);

  // Computations
  std::vector <pcl::PointIndices> computeSegments(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &colored_cloud);
  Mesh computeMesh();
  std::pair<int, int> getNextUnused(std::vector< std::pair<int,int> > used);
  int sortBoundary(pcl::IndicesPtr& boundary_indices, std::vector<pcl::IndicesPtr> &sorted_boundaries);
  void setSearchRadius(double radius);
  double getSearchRadius();

  // Smoothing
  bool setSmoothCoef(std::vector<double> &coef);
  void smoothVector(std::vector<double>&x_in, std::vector<double> &x_out);
  void smoothPointNormal(std::vector<pcl::PointNormal> &pts_in, std::vector<pcl::PointNormal> &pts_out);

  // Misc

  void getBoundaryTrajectory(std::vector<pcl::IndicesPtr> &boundaries,
                             int sb,
                             std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> &poses);

private:
  void removeNans();
  void computeNormals();

  pcl::PointCloud<pcl::Normal>::Ptr normals_;
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals_;

};
#endif
