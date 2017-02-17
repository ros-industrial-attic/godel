#include <meshing_plugins/concave_hull_plugins.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pluginlib/class_list_macros.h>
#include <meshing_plugins_base/meshing_base.h>

const static double CONCAVE_HULL_ALPHA = 0.1;

namespace concave_hull_mesher
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  void ConcaveHullMesher::init(PointCloud input)
  {
    input_cloud_ = input;
  }

  bool ConcaveHullMesher::generateMesh(pcl::PolygonMesh& mesh)
  {
    pcl::ConcaveHull<Point> concave_hull;
    pcl::EarClipping ear_clipping;
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);

    concave_hull.setInputCloud(input_cloud_.makeShared());
    concave_hull.setAlpha(CONCAVE_HULL_ALPHA);
    concave_hull.reconstruct(*mesh_ptr);

    ear_clipping.setInputMesh(mesh_ptr);
    ear_clipping.process(mesh);

    return mesh.polygons.size() > 0;
  }
} // end mesher_plugins

PLUGINLIB_EXPORT_CLASS(concave_hull_mesher::ConcaveHullMesher, meshing_plugins_base::MeshingBase)
