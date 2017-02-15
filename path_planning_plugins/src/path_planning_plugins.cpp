#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include "path_planning_plugins.h"
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>


const static std::string DEFAULT_PARAM_PREFIX = "/path_planning_params/";
const static std::string DISCRETIZATION = DEFAULT_PARAM_PREFIX + "discretization";
const static std::string MARGIN = DEFAULT_PARAM_PREFIX + "margin";
const static std::string OVERLAP = DEFAULT_PARAM_PREFIX + "overlap";
const static std::string SAFE_TRAVERSE_HEIGHT = DEFAULT_PARAM_PREFIX + "safe_traverse_height";
const static std::string SCAN_WIDTH = DEFAULT_PARAM_PREFIX + "scan_width";
const static std::string TOOL_RADIUS = DEFAULT_PARAM_PREFIX + "tool_radius";
const static double MIN_BOUNDARY_LENGTH = 0.1; // 10 cm
const static std::string PATH_GENERATION_SERVICE = "process_path_generator";

// Together these constants define a 5cm approach and departure path for the laser scans
const static int SCAN_APPROACH_STEP_COUNT = 5;
const static double SCAN_APPROACH_STEP_DISTANCE = 0.01; // 1cm

namespace path_planning_plugins
{
  bool Openveronoi::populatePlanningParameters(ros::NodeHandle& nh, path_planning_base::PlanningParams& params)
  {
    try
    {
      nh.getParam(DISCRETIZATION, params.discretization);
      nh.getParam(MARGIN, params.margin);
      nh.getParam(OVERLAP, params.overlap);
      nh.getParam(SAFE_TRAVERSE_HEIGHT, params.traverse_height);
      nh.getParam(SCAN_WIDTH, params.scan_width);
      nh.getParam(TOOL_RADIUS, params.tool_radius);
      return true;
    }
    catch(const std::exception& e)
    {
      ROS_ERROR_STREAM("Unable to populate path planning parameters" << e.what());
    }
    return false;
  }

  static godel_process_path::PolygonBoundaryCollection
  filterPolygonBoundaries(const godel_process_path::PolygonBoundaryCollection& boundaries)
  {
    godel_process_path::PolygonBoundaryCollection filtered_boundaries;

    for (std::size_t i = 0; i < boundaries.size(); ++i)
    {
      const godel_process_path::PolygonBoundary& bnd = boundaries[i];
      double circ = godel_process_path::polygon_utils::circumference(bnd);

      if (circ < MIN_BOUNDARY_LENGTH)
      {
        ROS_WARN_STREAM("Ignoring boundary with length " << circ);
      }
      else if (!godel_process_path::polygon_utils::checkBoundary(bnd))
      {
        ROS_WARN_STREAM("Ignoring ill-formed boundary");
      }
      else
      {
        filtered_boundaries.push_back(bnd);
        godel_process_path::polygon_utils::filter(filtered_boundaries.back(), 0.1);
        std::reverse(filtered_boundaries.back().begin(), filtered_boundaries.back().end());
      }
    }
    return filtered_boundaries;
  }



  // Blend Planner

  void Openveronoi::BlendPlanner::init(pcl::PolygonMesh mesh)
  {
    mesh_ = mesh;
    mesh_importer_ = new mesh_importer::MeshImporter(false);
    process_path_client_ = nh_.serviceClient<godel_msgs::PathPlanning>(PATH_GENERATION_SERVICE);
  }

  bool Openveronoi::BlendPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
  {
    path.clear();
    if (populatePlanningParameters(nh_, params_))
    {
      using godel_process_path::PolygonBoundaryCollection;
      using godel_process_path::PolygonBoundary;

      // Calculate boundaries for a surface
      if (mesh_importer_.calculateSimpleBoundary(mesh_))
      {
        // Read & filter boundaries that are ill-formed or too small
        PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_.getBoundaries());

        // Read pose
        geometry_msgs::Pose boundary_pose;
        mesh_importer_.getPose(boundary_pose);

        // Send request to blend path generation service
        godel_msgs::PathPlanning srv;
        srv.request.params = params_;
        godel_process_path::utils::translations::godelToGeometryMsgs(srv.request.surface.boundaries, filtered_boundaries);
        tf::poseTFToMsg(tf::Transform::getIdentity(), srv.request.surface.pose);

        if (!process_path_client_.call(srv))
        {
          ROS_ERROR_STREAM("Process path cilent failed");
          return false;
        }

        // blend process path calculations suceeded. Save data into results.
        geometry_msgs::PoseArray blend_poses;
        geometry_msgs::PoseArray path_local = srv.response.poses;
        geometry_msgs::Pose p;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;

        // Transform points to world frame and generate pose
        Eigen::Affine3d boundary_pose_eigen;
        Eigen::Affine3d eigen_p;
        Eigen::Affine3d result;

        tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

        for (const auto& pose : path_local.poses)
        {
          p.position.x = pose.position.x;
          p.position.y = pose.position.y;
          p.position.z = pose.position.z;

          tf::poseMsgToEigen(p, eigen_p);
          result = boundary_pose_eigen*eigen_p;
          tf::poseEigenToMsg(result, p);
          p.orientation = boundary_pose.orientation;
          blend_poses.poses.push_back(p);
        }

        path.push_back(blend_poses);
        return true;
      }
      else
        ROS_WARN_STREAM("Could not calculate boundary for mesh");

      return false;
    }
    else
      return false;
  }


  // Scan Planner

  void Openveronoi::ScanPlanner::init(pcl::PolygonMesh mesh)
  {
    mesh_ = mesh;
    mesh_importer_ = new mesh_importer::MeshImporter(false);
  }

  bool Openveronoi::ScanPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
  {
    path.clear();
    if (populatePlanningParameters(nh_, params_))
    {
      using godel_process_path::PolygonBoundaryCollection;
      using godel_process_path::PolygonBoundary;

      // 0 - Calculate boundaries for a surface
      if (mesh_importer_.calculateSimpleBoundary(mesh_))
      {
        // 1 - Read & filter boundaries that are ill-formed or too small
        PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_.getBoundaries());

        // 2 - Read boundary pose
        geometry_msgs::Pose boundary_pose;
        mesh_importer_.getPose(boundary_pose);

        // 3 - Skip if boundaries are empty
        if (filtered_boundaries.empty())
          return false;

        geometry_msgs::PoseArray scan_poses;

        // 4 - Generate scan polygon boundary
        PolygonBoundary scan = scan::generateProfilometerScanPath(filtered_boundaries.front(), params_);

        // 5 - Get boundary pose eigen
        Eigen::Affine3d boundary_pose_eigen;
        tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

        // 6 - Transform points to world frame and generate pose
      std::vector<geometry_msgs::Point> points;

        for(const auto& pt : scan)
        {
        geometry_msgs::Point p;
          p.x = pt.x;
          p.y = pt.y;
          p.z = 0.0;

          points.push_back(p);
        }

      std::transform(points.begin(), points.end(), std::back_inserter(scan_poses.poses),
                     [boundary_pose_eigen] (const geometry_msgs::Point& point) {
        geometry_msgs::Pose pose;
        Eigen::Affine3d r = boundary_pose_eigen * Eigen::Translation3d(point.x, point.y, point.z);
        tf::poseEigenToMsg(r, pose);
        return pose;
      });

        // 7 - Add in approach and departure
        geometry_msgs::PoseArray approach;
        geometry_msgs::Pose start_pose;
        geometry_msgs::Pose end_pose;
        start_pose = scan_poses.poses.front();
        end_pose = scan_poses.poses.back();
        double start_z = start_pose.position.z;
        double end_z = end_pose.position.z;

        for (std::size_t i = 0; i < SCAN_APPROACH_STEP_COUNT; ++i)
        {
          double z_offset = i * SCAN_APPROACH_STEP_DISTANCE;
          geometry_msgs::Pose approach_pose;
          geometry_msgs::Pose departure_pose;

          approach_pose.orientation = start_pose.orientation;
          approach_pose.position = start_pose.position;
          approach_pose.position.z = start_z + z_offset;
          scan_poses.poses.insert(scan_poses.poses.begin(), approach_pose);

          departure_pose.orientation = end_pose.orientation;
          departure_pose.position = end_pose.position;
          departure_pose.position.z = end_z + z_offset;
          scan_poses.poses.push_back(departure_pose);
        }

        // 8 - return result
        path.push_back(scan_poses);
        return true;
      }
      else
        ROS_WARN_STREAM("Could not calculate boundary for mesh");
      return false;
    }
    else
      return false;
  }
} // end mesher_plugins

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::Openveronoi::BlendPlanner, path_planning_base::PathPlanningBase)

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::Openveronoi::ScanPlanner, path_planning_base::PathPlanningBase)
