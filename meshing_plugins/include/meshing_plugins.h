#ifndef MESHING_PLUGINS_H
#define MESHING_PLUGINS_H

#include <meshing_base.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <geometry_msgs/PoseArray.h>

const static double CONCAVE_HULL_ALPHA = 0.1;

namespace meshing_plugins
{
  class ConcaveHullMesher : public meshing_base::MeshingBase
  {
  private:
    pcl::ConcaveHull<meshing_base::Point> chull_;
    pcl::EarClipping clipping_;
    meshing_base::PointCloud input_cloud_;

  public:
    ConcaveHullMesher(){}
    void init(meshing_base::PointCloud input);
    bool generateMesh(pcl::PolygonMesh& mesh);
  };
}

#endif // MESHER_PLUGINS_H
