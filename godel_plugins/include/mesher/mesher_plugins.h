#ifndef MESHER_PLUGINS_H
#define MESHER_PLUGINS_H

#include <mesher/mesher_base.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <geometry_msgs/PoseArray.h>

const static double CONCAVE_HULL_ALPHA = 0.1;

namespace mesher_plugins
{
  class ConcaveHullMesher : public mesher_base::MesherBase
  {
  private:
    pcl::ConcaveHull<mesher_base::Point> chull_;
    pcl::EarClipping clipping_;
    mesher_base::PointCloud input_cloud_;

  public:
    ConcaveHullMesher(){}
    void init(mesher_base::PointCloud input);
    bool generateMesh(pcl::PolygonMesh& mesh);
  };
}

#endif // MESHER_PLUGINS_H
