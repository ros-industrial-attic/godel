#ifndef DATA_COORDINATOR_H
#define DATA_COORDINATOR_H

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PoseArray.h>
#include <ros/io.h>

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

    /**
     * @brief getNextID return a unique ID
     * @return
     */
    int getNextID()
    {
      return ++id_counter_;
    }


    /**
     * @brief printIds
     * @return a formatted string containing all current record IDs
     */
    std::string printIds()
    {
      std::stringstream ss;
      std::string separator = "[";
      for(const auto& rec : records_)
      {
        ss << separator << rec.id_;
        separator = ", ";
      }
      ss << "]";
      return ss.str();
    }


  public:
    //! Default Constructor
    DataCoordinator()
    {
      id_counter_ = 0;
      if(records_.size() > 0)
        records_.clear();
    }

    /**
     * @brief init Initializes DataCoordinator object
     * @return success of operation
     */
    bool init()
    {
      id_counter_ = 0;
      if(records_.size() > 0)
        records_.clear();
      return true;
    }


    /**
     * @brief Generates an id and creates a SurfaceDetectionRecord which connects detection features
     * @param input_cloud source point cloud from which the surface was derived
     * @param surface_cloud point cloud which desribes the surface
     * @param id of the new record
     */
    int addRecord(pcl::PointCloud<pcl::PointXYZRGB> input_cloud, pcl::PointCloud<pcl::PointXYZRGB> surface_cloud)
    {
      SurfaceDetectionRecord rec;
      rec.id_ = getNextID();
      rec.input_cloud_ = input_cloud;
      rec.surface_cloud_ = surface_cloud;
      records_.push_back(rec);
      ROS_INFO_STREAM("addRecord called with id: " << rec.id_);
      return rec.id_;
    }


    /**
     * @brief getCloud Returns a cloud of interest
     * @param type Type of cloud to return (currently implemented: input, surface)
     * @param id ID of the desired record
     * @param cloud Destination for cloud
     * @return true if record is found and type is valid, false otherwise
     */
    bool getCloud(int type, int id, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
    {
      ROS_INFO_STREAM("In data_coordinator getCloud");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          switch(type)
          {
            case INPUT_CLOUD_TYPE:
            {
              cloud = rec.input_cloud_;
              return true;
            }

            case SURFACE_CLOUD_TYPE:
            {
              cloud = rec.surface_cloud_;
              return true;
            }

            default:
            {
              ROS_WARN_STREAM("Invalid cloud type");
              return false;
            }
          }
        }
      }

      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }


    /**
     * @brief setSurfaceName Adds/Mutates surface name in record
     * @param id ID of the desired record
     * @param name Desired name of the surface
     * @return true if record is found, false otherwise
     */
    bool setSurfaceName(int id, const std::string& name)
    {
      ROS_INFO_STREAM("In data_coordinator setSurfaceName");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          ROS_INFO_STREAM("Setting surface name to: " << name);
          rec.surface_name_ = name;
          return true;
        }
      }

      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }


    /**
     * @brief getSurfaceName Returns the surface name from a record
     * @param id ID of the desired record
     * @param name Destination variable for surface name
     */
    void getSurfaceName(int id, std::string& name)
    {
      ROS_INFO_STREAM("In data_coordinator getSurfaceName");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          name = rec.surface_name_;
          break;
        }
      }
    }


    /**
     * @brief Set the mesh of the specified record
     * @param id ID of the desired record
     * @param mesh PolygonMesh representing the surface of interest
     * @return true if record is found, false otherwise
     */
    bool setSurfaceMesh(int id, pcl::PolygonMesh mesh)
    {
      ROS_INFO_STREAM("In data_coordinator setSurfaceMesh");
      ROS_INFO_STREAM("Attempting to add mesh for record with id: " << id << " from " << printIds());
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          rec.surface_mesh_ = mesh;
          return true;
        }
      }

      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }


    /**
     * @brief getSurfaceMesh
     * @param id ID of the desired record
     * @param mesh Destination for the mesh
     */
    void getSurfaceMesh(int id, pcl::PolygonMesh& mesh)
    {
      ROS_INFO_STREAM("In data_coordinator getSurfaceMesh");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          mesh = rec.surface_mesh_;
          break;
        }
      }
    }


    /**
     * @brief Add poses comprising the edge of a surface to its record
     * @param id ID of the desired record
     * @param name The name of the edge
     * @param edge_poses PoseArray of edges
     * @return true if record is found and edge is added, false otherwise
     */
    bool addEdge(int id, std::string name, geometry_msgs::PoseArray edge_poses)
    {
      ROS_INFO_STREAM("In data_coordinator addEdge");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          std::pair<std::string, geometry_msgs::PoseArray> edge_pair;
          edge_pair.first = name;
          edge_pair.second = edge_poses;
          rec.edge_pairs_.push_back(edge_pair);
          return true;
        }
      }

      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }


    /**
     * @brief renameEdge Change name for an edge
     * @param id ID of the desired record
     * @param old_name
     * @param new_name
     * @return true if both record is found and an edge with old_name exists within the record, false otherwise
     */
    bool renameEdge(int id, std::string old_name, std::string new_name)
    {
      ROS_INFO_STREAM("In data_coordinator renameEdge");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          for(auto& pair: rec.edge_pairs_)
          {
            if(old_name.compare(new_name) == 0)
            {
              pair.first = new_name;
              return true;
            }
          }

          ROS_WARN_STREAM("Unable to find edge path with name: " << old_name);
          return false;
        }
      }

      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }


    /**
     * @brief setPoses Add/change poses in a record
     * @param type Type of pose to add (currently implemented: blend, scan)
     * @param id ID of the desired record
     * @param poses PoseArray containing pose data
     * @return true if record is found, false otherwise
     */
    bool setPoses(int type, int id, geometry_msgs::PoseArray poses)
    {
      ROS_INFO_STREAM("In data_coordinator setPoses");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          if(type == BLEND_POSE_TYPE)
          {
            rec.blend_poses_ = poses;
            return true;
          }
          else if(type == SCAN_POSE_TYPE)
          {
            rec.scan_poses_ = poses;
            return true;
          }
          else
          {
            ROS_WARN_STREAM("Unknown type for setPoses: " << type);
            return false;
          }
        }

      }
      ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }

    /**
     * @brief getPoses Get PoseArray with path plan
     * @param type Type of pose to retrieve (currently implemented, blend, scan)
     * @param id ID of the desired record
     * @param poses
     * @return
     */
    bool getPoses(int type, int id, geometry_msgs::PoseArray& poses)
    {
      ROS_INFO_STREAM("In data_coordinator getPoses");
      for(auto& rec : records_)
      {
        if(id == rec.id_)
        {
          switch(type)
          {
          case BLEND_POSE_TYPE:
            poses = rec.blend_poses_;
            return true;
          case SCAN_POSE_TYPE:
            poses = rec.scan_poses_;
            return true;
          }
        }
      }
      ROS_WARN_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
      return false;
    }

  };
} /* end namespace data */
} /* end namespace godel_surface_detection */


#endif // DATA_COORDINATOR_H
