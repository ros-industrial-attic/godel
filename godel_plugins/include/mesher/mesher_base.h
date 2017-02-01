#ifndef MESHER_BASE_H_
#define MESHER_BASE_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>

namespace mesher_base
{
  typedef pcl::PointXYZRGB Point;
  typedef pcl::PointCloud<Point> PointCloud;

  class MesherBase
  {
  public:
    /**
     * @brief Initialize the mesher
     * @details [long description]
     * 
     * @param  pcl::PointCloud input cloud for the mesher
     */
    virtual void init(PointCloud) = 0;

    /**
     * @brief Generates a pcl::PolygonMesh from the input PointCloud
     * @details [long description]
     * @param  [description]
     * @return true if the generation was successful, false if it failed
     */
    virtual bool generateMesh(pcl::PolygonMesh&) = 0;
  };
} // end mesher_base

#endif
