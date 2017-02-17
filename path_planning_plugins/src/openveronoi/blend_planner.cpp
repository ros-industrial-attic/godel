#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <godel_msgs/PathPlanning.h>
#include <mesh_importer/mesh_importer.h>
#include <path_planning_plugins/openveronoi_plugins.h>
#include <pluginlib/class_list_macros.h>
#include <ros/node_handle.h>
#include <tf/transform_datatypes.h>

const static std::string PATH_GENERATION_SERVICE = "process_path_generator";


namespace path_planning_plugins
{
typedef  godel_msgs::PathPlanningParameters PlanningParams;

void openveronoi::BlendPlanner::init(pcl::PolygonMesh mesh)
{
  mesh_ = mesh;
}

bool openveronoi::BlendPlanner::generatePath(std::vector<geometry_msgs::PoseArray>& path)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  path.clear();

  std::unique_ptr<mesh_importer::MeshImporter> mesh_importer_ptr(new mesh_importer::MeshImporter(false));
  ros::NodeHandle nh;
  ros::ServiceClient process_path_client = nh.serviceClient<godel_msgs::PathPlanning>(PATH_GENERATION_SERVICE);
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


  // Calculate boundaries for a surface
  if (mesh_importer_ptr->calculateSimpleBoundary(mesh_))
  {
    // Read & filter boundaries that are ill-formed or too small
    PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_ptr->getBoundaries());

    // Read pose
    geometry_msgs::Pose boundary_pose;
    mesh_importer_ptr->getPose(boundary_pose);

    // Send request to blend path generation service
    godel_msgs::PathPlanning srv;
    srv.request.params = params;
    godel_process_path::utils::translations::godelToGeometryMsgs(srv.request.surface.boundaries, filtered_boundaries);
    tf::poseTFToMsg(tf::Transform::getIdentity(), srv.request.surface.pose);

    if (!process_path_client.call(srv))
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
} // end namespace

PLUGINLIB_EXPORT_CLASS(path_planning_plugins::openveronoi::BlendPlanner, path_planning_plugins_base::PathPlanningBase)
