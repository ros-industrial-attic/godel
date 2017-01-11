#ifndef DATA_COORDINATOR_H
#define DATA_COORDINATOR_H

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseArray.h>

namespace godel_surface_detection
{
namespace data
{
  // Pose Types
  const static int BLEND_POSE_TYPE = 1;
  const static int SCAN_POSE_TYPE = 2;

  // Cloud Types
  const static int INPUT_CLOUD_TYPE = 1;
  const static int SURFACE_CLOUD_TYPE = 2;
  const static int BOUNDARY_CLOUD_TYPE = 3;  // for future use

  // Common Error Strings
  const static std::string UNABLE_TO_FIND_RECORD_ERROR = "Unable to get record with specified id";

  /**
   * @brief A structure containing features pertinent to surface detection.
   */
  struct SurfaceDetectionRecord
  {
    public:
      int id_;
      std::string surface_name_;
      pcl::PointCloud<pcl::PointXYZRGB> input_cloud_;
      pcl::PolygonMesh surface_mesh_;
      pcl::PointCloud<pcl::PointXYZRGB> surface_cloud_;
      std::vector<std::pair<std::string, geometry_msgs::PoseArray>> edge_pairs_;
      geometry_msgs::PoseArray blend_poses_;
      geometry_msgs::PoseArray scan_poses_;
  };


  /**
   * @brief Class to handle acquiring and diseminating data relevant to surface detection features
   */
  class DataCoordinator
  {
  private:
    int id_counter_ = 0;
    std::vector<SurfaceDetectionRecord> records_;

    int getNextID();
    std::string printIds();

  public:
    DataCoordinator();
    bool init();
    int addRecord(pcl::PointCloud<pcl::PointXYZRGB> input_cloud, pcl::PointCloud<pcl::PointXYZRGB> surface_cloud);
    bool getCloud(int type, int id, pcl::PointCloud<pcl::PointXYZRGB>& cloud);
    bool setSurfaceName(int id, const std::string& name);
    void getSurfaceName(int id, std::string& name);
    bool setSurfaceMesh(int id, pcl::PolygonMesh mesh);
    void getSurfaceMesh(int id, pcl::PolygonMesh& mesh);
    bool addEdge(int id, std::string name, geometry_msgs::PoseArray edge_poses);
    bool renameEdge(int id, std::string old_name, std::string new_name);
    bool getEdgePosesByName(const std::string& edge_name, geometry_msgs::PoseArray& edge_poses);
    bool setPoses(int type, int id, geometry_msgs::PoseArray poses);
    bool getPoses(int type, int id, geometry_msgs::PoseArray& poses);
  };
} /* end namespace data */
} /* end namespace godel_surface_detection */


#endif // DATA_COORDINATOR_H
