#include <coordination/data_coordinator.h>
#include <ros/io.h>

namespace godel_surface_detection
{
namespace data
{

  /**
   * @brief getNextID return a unique ID
   * @return
   */
  int DataCoordinator::getNextID()
  {
    return ++id_counter_;
  }


  /**
   * @brief printIds
   * @return a formatted string containing all current record IDs
   */
  std::string DataCoordinator::printIds()
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


  //! Default Constructor
  DataCoordinator::DataCoordinator()
  {
    id_counter_ = 0;
    if(records_.size() > 0)
      records_.clear();
  }

  /**
   * @brief init Initializes DataCoordinator object
   * @return success of operation
   */
  bool DataCoordinator::init()
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
  int DataCoordinator::addRecord(pcl::PointCloud<pcl::PointXYZRGB> input_cloud,
                                 pcl::PointCloud<pcl::PointXYZRGB> surface_cloud)
  {
    SurfaceDetectionRecord rec;
    rec.id_ = getNextID();
    rec.input_cloud_ = input_cloud;
    rec.surface_cloud_ = surface_cloud;
    records_.push_back(rec);
    return rec.id_;
  }


  /**
   * @brief getCloud Returns a cloud of interest
   * @param type Type of cloud to return (currently implemented: input, surface)
   * @param id ID of the desired record
   * @param cloud Destination for cloud
   * @return true if record is found and type is valid, false otherwise
   */
  bool DataCoordinator::getCloud(CloudTypes cloud_type, int id, pcl::PointCloud<pcl::PointXYZRGB>& cloud)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
        switch(cloud_type)
        {
          case input_cloud:
          {
            cloud = rec.input_cloud_;
            return true;
          }

          case surface_cloud:
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
  bool DataCoordinator::setSurfaceName(int id, const std::string& name)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
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
   * @return ture if record is found, false otherwise
   */
  bool DataCoordinator::getSurfaceName(int id, std::string& name)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
        name = rec.surface_name_;
        return true;
      }
    }

    ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
    return false;
  }


  /**
   * @brief Set the mesh of the specified record
   * @param id ID of the desired record
   * @param mesh PolygonMesh representing the surface of interest
   * @return true if record is found, false otherwise
   */
  bool DataCoordinator::setSurfaceMesh(int id, pcl::PolygonMesh mesh)
  {
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
   * @return true if record is found, false otherwise
   */
  bool DataCoordinator::getSurfaceMesh(int id, pcl::PolygonMesh& mesh)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
        mesh = rec.surface_mesh_;
        return true;
      }
    }

    ROS_ERROR_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
    return false;
  }


  /**
   * @brief Add poses comprising the edge of a surface to its record
   * @param id ID of the desired record
   * @param name The name of the edge
   * @param edge_poses PoseArray of edges
   * @return true if record is found and edge is added, false otherwise
   */
  bool DataCoordinator::addEdge(int id, std::string name, geometry_msgs::PoseArray edge_poses)
  {
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
  bool DataCoordinator::renameEdge(int id, std::string old_name, std::string new_name)
  {
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
   * @brief getEdgePosesByName Obtains PoseArray for edge with name edge_name
   * @param edge_name
   * @param edge_poses Destination for PoseArray
   * @return true if record is found and an edge with edge_name exists within the record, false otherwise
   */
  bool DataCoordinator::getEdgePosesByName(const std::string& edge_name, geometry_msgs::PoseArray& edge_poses)
  {
    for(auto& rec : records_)
    {
      for (auto& edge_pair : rec.edge_pairs_)
      {
        if(edge_name.compare(edge_pair.first) == 0)
        {
          edge_poses = edge_pair.second;
          return true;
        }
      }
    }

    ROS_WARN_STREAM("Unable to find edge path with name: " << edge_name);
    return false;
  }


  /**
   * @brief setPoses Add/change poses in a record
   * @param type Type of pose to add (currently implemented: blend, scan)
   * @param id ID of the desired record
   * @param poses PoseArray containing pose data
   * @return true if record is found, false otherwise
   */
  bool DataCoordinator::setPoses(PoseTypes pose_type, int id, const std::vector<geometry_msgs::PoseArray>& poses)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
        switch(pose_type)
        {
          case blend_pose:
          {
            rec.blend_poses_ = poses;
            return true;
          }

          case scan_pose:
          {
            rec.scan_poses_ = poses;
            return true;
          }

          default:
          {
            ROS_WARN_STREAM("Unknown type for setPoses: " << pose_type);
            return false;
          }
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
  bool DataCoordinator::getPoses(PoseTypes pose_type, int id, std::vector<geometry_msgs::PoseArray>& poses)
  {
    for(auto& rec : records_)
    {
      if(id == rec.id_)
      {
        switch(pose_type)
        {
          case blend_pose:
          {
            poses = rec.blend_poses_;
            return true;
          }

          case scan_pose:
          {
            poses = rec.scan_poses_;
            return true;
          }

          default:
          {
            ROS_WARN_STREAM("Unrecognized pose type");
            return false;
          }
        }
      }
    }

    ROS_WARN_STREAM(UNABLE_TO_FIND_RECORD_ERROR << " " << id);
    return false;
  }
} /* end namespace data */
} /* end namespace godel_surface_detection */
