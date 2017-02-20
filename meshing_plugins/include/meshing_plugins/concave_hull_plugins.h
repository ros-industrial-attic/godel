#ifndef CONCAVE_HULL_PLUGINS_H
#define CONCAVE_HULL_PLUGINS_H

#include <meshing_plugins_base/meshing_base.h>

namespace concave_hull_mesher
{
  class ConcaveHullMesher : public meshing_plugins_base::MeshingBase
  {
  private:
    pcl::PointCloud<pcl::PointXYZRGB> input_cloud_;

  public:
    ConcaveHullMesher(){}
    void init(pcl::PointCloud<pcl::PointXYZRGB> input);
    bool generateMesh(pcl::PolygonMesh& mesh);
  };
}

#endif // CONCAVE_HULL_PLUGINS_H
