#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include <mesh_importer/mesh_importer.h>
#include <path_planning_plugins/openveronoi_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <profilometer/profilometer_scan.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

// Together these constants define a 5cm approach and departure path for the laser scans
const static int SCAN_APPROACH_STEP_COUNT = 5;
const static double SCAN_APPROACH_STEP_DISTANCE = 0.01; // 1cm

namespace path_planning_plugins
{
// Scan Planner

void openveronoi::ScanPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}

bool openveronoi::ScanPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  path.clear();

  std::unique_ptr<mesh_importer::MeshImporter> mesh_importer_ptr(new mesh_importer::MeshImporter(false));

  ros::NodeHandle nh;
  godel_msgs::PathPlanningParameters params;
  try
  {
    nh.getParam(DISCRETIZATION, params.discretization);
    nh.getParam(MARGIN, params.margin);
    nh.getParam(OVERLAP, params.overlap);
    nh.getParam(SAFE_TRAVERSE_HEIGHT, params.traverse_height);
    nh.getParam(SCAN_WIDTH, params.scan_width);
    nh.getParam(TOOL_RADIUS, params.tool_radius);
  }
  catch(const std::exception& e)
  {
    ROS_ERROR_STREAM("Unable to populate path planning parameters" << e.what());
    return false;
  }

  // 0 - Calculate boundaries for a surface
  if (mesh_importer_ptr->calculateSimpleBoundary(mesh_))
  {
    // 1 - Read & filter boundaries that are ill-formed or too small
    PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_ptr->getBoundaries());

    // 2 - Read boundary pose
    geometry_msgs::Pose boundary_pose;
    mesh_importer_ptr->getPose(boundary_pose);

    // 3 - Skip if boundaries are empty
    if (filtered_boundaries.empty())
      return false;

    geometry_msgs::PoseArray scan_poses;

    // 4 - Generate scan polygon boundary
    PolygonBoundary scan_boundary = scan::generateProfilometerScanPath(filtered_boundaries.front(), params);

    // 5 - Get boundary pose eigen
    Eigen::Affine3d boundary_pose_eigen;
    tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

    // 6 - Transform points to world frame and generate pose
    std::vector<geometry_msgs::Point> points;

    for(const auto& pt : scan_boundary)
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

    // 8 - return result
    path.push_back(scan_poses);
    return true;
  }
  else
    ROS_WARN_STREAM("Could not calculate boundary for mesh");
  return false;
}
} // end namespace path_planning_plugins

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::openveronoi::ScanPlanner, path_planning_plugins_base::PathPlanningBase)
