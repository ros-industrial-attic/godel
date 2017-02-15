#include <meshing_plugins.h>
#include <pluginlib/class_list_macros.h>

namespace meshing_plugins
{
  void ConcaveHullMesher::init(meshing_base::PointCloud input)
  {
    input_cloud_ = input;
  }

  bool ConcaveHullMesher::generateMesh(pcl::PolygonMesh& mesh)
  {
    pcl::PolygonMesh::Ptr mesh_ptr (new pcl::PolygonMesh);
    chull_.setInputCloud(input_cloud_.makeShared());
    chull_.setAlpha(CONCAVE_HULL_ALPHA);
    chull_.reconstruct(*mesh_ptr);
    clipping_.setInputMesh(mesh_ptr);
    clipping_.process(mesh);

    return mesh.polygons.size() > 0;
  }
} // end mesher_plugins

PLUGINLIB_EXPORT_CLASS(meshing_plugins::ConcaveHullMesher, meshing_base::MeshingBase)
