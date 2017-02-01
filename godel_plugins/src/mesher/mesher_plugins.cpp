#include <mesher/mesher_plugins.h>
#include <pluginlib/class_list_macros.h>

namespace mesher_plugins
{
  void ConcaveHullMesher::init(mesher_base::PointCloud input)
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

PLUGINLIB_EXPORT_CLASS(mesher_plugins::ConcaveHullMesher, mesher_base::MesherBase)
