#ifndef MESHING_PLUGINS_BASE_H_
#define MESHING_PLUGINS_BASE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace meshing_plugins_base
{
  class MeshingBase
  {
  public:
    /**
     * @brief Initialize the mesher
     * @param  pcl::PointCloud input cloud for the mesher
     */
    virtual void init(pcl::PointCloud<pcl::PointXYZRGB> input) = 0;

    /**
     * @brief Generates a pcl::PolygonMesh from the input PointCloud
     * @param  mesh: destination variable for the resulting mesh
     * @return true if the generation was successful, false if it failed
     */
    virtual bool generateMesh(pcl::PolygonMesh& mesh) = 0;
  };
} // end namespace meshing_plugins_base

#endif // end MESHING_PLUGINS_BASE_H_
