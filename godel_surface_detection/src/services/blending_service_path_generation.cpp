#include <services/surface_blending_service.h>
#include <segmentation/surface_segmentation.h>
#include <segmentation/edge_refinement.h>
#include <detection/surface_detection.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// Temporary constants for storing blending path `planning parameters
// Will be replaced by loadable, savable parameters
const static std::string BLEND_TRAJECTORY_BAGFILE = "blend_trajectory.bag";
const static std::string BLEND_TRAJECTORY_GROUP_NAME = "manipulator_tcp";
const static std::string BLEND_TRAJECTORY_TOOL_FRAME = "tcp_frame";
const static std::string BLEND_TRAJECTORY_WORLD_FRAME = "world_frame";
const static double BLEND_TRAJECTORY_ANGLE_DISC = M_PI / 10.0;

// Temporary constants for storing scan path planning parameters
// Will be replaced by loadable, savable parameters
const static std::string SCAN_TRAJECTORY_BAGFILE = "scan_trajectory.bag";
const static std::string SCAN_TRAJECTORY_GROUP_NAME = "manipulator_keyence";
const static std::string SCAN_TRAJECTORY_TOOL_FRAME = "keyence_tcp_frame";
const static std::string SCAN_TRAJECTORY_WORLD_FRAME = "world_frame";
const static double SCAN_TRAJECTORY_ANGLE_DISC = 0.2;

const static std::string BLEND_TYPE = "blend";
const static std::string EDGE_TYPE = "edge";
const static std::string SCAN_TYPE = "scan";

// Together these constants define a 5cm approach and departure path for the
// laser scans
const static int SCAN_APPROACH_STEP_COUNT = 5;
const static double SCAN_APPROACH_STEP_DISTANCE = 0.01; // 5cm

// Edge Processing constants
const static double SEGMENTATION_SEARCH_RADIUS = 0.030; // 3cm
const static int BOUNDARY_THRESHOLD = 10;

// Variables to select path type
const static int PATH_TYPE_BLENDING = 0;
const static int PATH_TYPE_SCAN = 1;
const static int PATH_TYPE_EDGE = 2;

const static std::string SURFACE_DESIGNATION = "surface_marker_server_";

/**
 * Prototype ProcessPlan refactoring - make it compatible with trajectory library and GUI
 */
static godel_process_path::PolygonBoundaryCollection
filterPolygonBoundaries(const godel_process_path::PolygonBoundaryCollection& boundaries,
                        const godel_msgs::BlendingPlanParameters& params)
{
  godel_process_path::PolygonBoundaryCollection filtered_boundaries;

  for (std::size_t i = 0; i < boundaries.size(); ++i)
  {
    const godel_process_path::PolygonBoundary& bnd = boundaries[i];
    double circ = godel_process_path::polygon_utils::circumference(bnd);

    if (circ < params.min_boundary_length)
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


bool SurfaceBlendingService::requestEdgePath(std::vector<pcl::IndicesPtr> &boundaries,
                                             int index,
                                             SurfaceSegmentation& SS,
                                             visualization_msgs::Marker& visualization,
                                             geometry_msgs::PoseArray& path)
{
  geometry_msgs::Pose geo_pose;
  std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;

  // Get boundary trajectory and trim last two poses (last poses are susceptible to large velocity changes)
  SS.getBoundaryTrajectory(boundaries, index, poses);
  poses.resize(poses.size() - 2);

  // Convert eigen poses to geometry poses for messaging and visualization
  for(const auto& p : poses)
  {
    Eigen::Affine3d pose(p.matrix());
    tf::poseEigenToMsg(pose, geo_pose);
    path.poses.push_back(geo_pose);
  }

  return true;
}

bool SurfaceBlendingService::requestBlendPath(
    const godel_process_path::PolygonBoundaryCollection& boundaries,
    const geometry_msgs::Pose& boundary_pose,
    const godel_msgs::BlendingPlanParameters& params,
    visualization_msgs::Marker&  visualization,
    geometry_msgs::PoseArray& path)
{
  godel_process_path_generation::VisualizeBlendingPlan srv;
  srv.request.params = params;
  godel_process_path::utils::translations::godelToGeometryMsgs(srv.request.surface.boundaries,
                                                               boundaries);
  tf::poseTFToMsg(tf::Transform::getIdentity(), srv.request.surface.pose);

  if (!visualize_process_path_client_.call(srv))
  {
    return false;
  }

  // blend process path calculations suceeded. Save data into results.
  visualization = srv.response.path;
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

  for (int i = 0; i < visualization.points.size(); i++)
  {
    p.position.x = visualization.points[i].x;
    p.position.y = visualization.points[i].y;
    p.position.z = visualization.points[i].z;

    tf::poseMsgToEigen(p, eigen_p);
    result = boundary_pose_eigen*eigen_p;
    tf::poseEigenToMsg(result, p);
    p.orientation = boundary_pose.orientation;
    path.poses.push_back(p);
  }

  visualization.ns = PATH_NAMESPACE;
  visualization.pose = boundary_pose;

  return true;
}

bool SurfaceBlendingService::requestScanPath(
    const godel_process_path::PolygonBoundaryCollection& boundaries,
    const geometry_msgs::Pose& boundary_pose,
    const godel_msgs::ScanPlanParameters& params,
    visualization_msgs::Marker& visualization,
    geometry_msgs::PoseArray& path)
{
  using namespace godel_process_path;
  using godel_process_path::utils::translations::godelToVisualizationMsgs;

  if (boundaries.empty())
    return false;

  PolygonBoundary scan =
      godel_surface_detection::generateProfilimeterScanPath(boundaries.front(), params);
  utils::translations::godelToVisualizationMsgs(visualization, scan, std_msgs::ColorRGBA());
  // Add in an approach and depart vector
  const geometry_msgs::Point& start_pt = visualization.points.front();
  const geometry_msgs::Point& end_pt = visualization.points.back();
  // Approach vector
  std::vector<geometry_msgs::Point> approach_points;
  for (std::size_t i = 0; i < SCAN_APPROACH_STEP_COUNT; ++i)
  {
    geometry_msgs::Point pt = start_pt;
    pt.z += (SCAN_APPROACH_STEP_COUNT - i) * SCAN_APPROACH_STEP_DISTANCE;
    approach_points.push_back(pt);
  }
  // Depart vector
  std::vector<geometry_msgs::Point> depart_points;
  for (std::size_t i = 0; i < SCAN_APPROACH_STEP_COUNT; ++i)
  {
    geometry_msgs::Point pt = end_pt;
    pt.z += i * SCAN_APPROACH_STEP_DISTANCE;
    depart_points.push_back(pt);
  }
  // Insert into path
  visualization.points.insert(visualization.points.end(), depart_points.begin(), depart_points.end());
  visualization.points.insert(visualization.points.begin(), approach_points.begin(), approach_points.end());

  geometry_msgs::Pose p;
  p.orientation.x = boundary_pose.orientation.x;
  p.orientation.y = boundary_pose.orientation.y;
  p.orientation.z = boundary_pose.orientation.z;
  p.orientation.w = boundary_pose.orientation.w;

  // Transform points to world frame and generate pose
  Eigen::Affine3d boundary_pose_eigen;
  Eigen::Affine3d eigen_p;
  Eigen::Affine3d result;

  tf::poseMsgToEigen(boundary_pose, boundary_pose_eigen);

  for(int i = 0; i < visualization.points.size(); i++)
  {
    p.position.x = visualization.points[i].x;
    p.position.y = visualization.points[i].y;
    p.position.z = visualization.points[i].z;

    tf::poseMsgToEigen(p, eigen_p);
    result = boundary_pose_eigen*eigen_p;
    tf::poseEigenToMsg(result, p);
    path.poses.push_back(p);
  }

  visualization.ns = PATH_NAMESPACE;
  visualization.pose = boundary_pose;

  return true;
}

void computeBoundaries(const godel_surface_detection::detection::CloudRGB::Ptr surface_cloud,
                       SurfaceSegmentation& SS,
                       std::vector< pcl::IndicesPtr>& sorted_boundaries)
{
  pcl::PointCloud<pcl::Boundary>::Ptr boundary_ptr (new pcl::PointCloud<pcl::Boundary>());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr boundary_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
  SS.getBoundaryCloud(boundary_ptr);
  int k=0;

  pcl::IndicesPtr boundary_idx(new std::vector<int>());
  for(const auto& pt : boundary_ptr->points)
  {
    if(pt.boundary_point)
    {
      boundary_cloud_ptr->points.push_back(surface_cloud->points[k]);
      boundary_idx->push_back(k);
    }
    k++;
  }

  boundary_cloud_ptr->width = 1;
  boundary_cloud_ptr->height = boundary_cloud_ptr->points.size();


  // sort the boundaries
  SS.sortBoundary(boundary_idx, sorted_boundaries);

  int max=0;
  int max_idx=0;

  for(int i=0;i<sorted_boundaries.size();i++)
  {
    if(sorted_boundaries[i]->size() > max)
    {
      max = sorted_boundaries[i]->size();
      max_idx = i;
    }
  }
}

inline static bool isBlendingPath(const std::string& name)
{
  const static std::string suffix("_blend");
  if (name.size() < suffix.size())
    return false;
  return name.find(suffix, name.size() - suffix.length()) != std::string::npos;
}


inline static bool isEdgePath(const std::string& name)
{
  const static std::string suffix("_edge");
  if (name.size() < suffix.size())
    return false;
  return name.find(suffix) != std::string::npos;
}

inline static bool isScanPath(const std::string& name)
{
  const static std::string suffix("_scan");
  if (name.size() < suffix.size())
    return false;
  return name.find(suffix) != std::string::npos;
}


ProcessPathResult
SurfaceBlendingService::generateProcessPath(const int& id,
                                            const godel_msgs::BlendingPlanParameters& blend_params,
                                            const godel_msgs::ScanPlanParameters& scan_params)
{
  using godel_surface_detection::detection::CloudRGB;

  std::string name;
  pcl::PolygonMesh mesh;
  CloudRGB::Ptr surface_ptr (new CloudRGB);

  data_coordinator_.getSurfaceName(id, name);
  data_coordinator_.getSurfaceMesh(id, mesh);
  data_coordinator_.getCloud(godel_surface_detection::data::CloudTypes::surface_cloud, id, *surface_ptr);

  return generateProcessPath(id, name, mesh, surface_ptr, blend_params, scan_params);
}


ProcessPathResult
SurfaceBlendingService::generateProcessPath(const int& id,
                                            const std::string& name,
                                            const pcl::PolygonMesh& mesh,
                                            godel_surface_detection::detection::CloudRGB::Ptr surface,
                                            const godel_msgs::BlendingPlanParameters& blend_params,
                                            const godel_msgs::ScanPlanParameters& scan_params)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;

  ProcessPathResult result;

  // Calculate boundaries for a surface
  if (!mesh_importer_.calculateSimpleBoundary(mesh))
  {
    ROS_WARN_STREAM("Could not calculate boundary for mesh associated with name: " << name);
    return result;
  }

  // Read & filter boundaries that are ill-formed or too small
  PolygonBoundaryCollection filtered_boundaries =
      filterPolygonBoundaries(mesh_importer_.getBoundaries(), blend_params);

  // Read pose
  geometry_msgs::Pose boundary_pose;
  mesh_importer_.getPose(boundary_pose);

  // Send request to blend path generation service
  visualization_msgs::Marker blend_visualization;
  geometry_msgs::PoseArray blend_poses;
  if (requestBlendPath(filtered_boundaries, boundary_pose, blend_params, blend_visualization, blend_poses))
  {
    ProcessPathResult::value_type blend_path_result; // pair<string, geometry_msgs::PoseArray>
    blend_path_result.first = name + "_blend";
    blend_path_result.second = blend_poses;
    result.paths.push_back(blend_path_result);

    // Hack for visualization sake
    blend_visualization.header.frame_id = "world_frame";
    blend_visualization.id = marker_counter_++;
    blend_visualization.header.stamp = ros::Time::now();
    blend_visualization.lifetime = ros::Duration(0.0);
    blend_visualization.ns = "blend_paths";
    blend_visualization.pose = boundary_pose;
    blend_visualization.action = visualization_msgs::Marker::ADD;
    blend_visualization.type = visualization_msgs::Marker::LINE_STRIP;
    blend_visualization.scale.x = 0.004;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.b = 0.0;
    color.g = 0.0;
    color.a = 1.0;
    blend_visualization.colors.clear();
    blend_visualization.color = color;

    process_path_results_.process_visualization_.markers.push_back(blend_visualization);
    data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::blend_pose, id, blend_poses);
  }
  else
  {
    // Blend path request failed
    ROS_WARN_STREAM("Could not calculate blend path for surface: " << name);
  }

  ROS_INFO_STREAM("Blend Path Generation Complete");

  // Send request to edge path generation service
  std::vector<pcl::IndicesPtr> sorted_boundaries;

  ROS_INFO_STREAM("Surface has " + std::to_string(surface->points.size()) + "points");
  // Compute the boundary
  SurfaceSegmentation SS(surface);

  SS.setSearchRadius(SEGMENTATION_SEARCH_RADIUS);
  std::vector<double> filt_coef;
  filt_coef.push_back(1);
  filt_coef.push_back(2);
  filt_coef.push_back(3);
  filt_coef.push_back(4);
  filt_coef.push_back(5);
  filt_coef.push_back(4);
  filt_coef.push_back(3);
  filt_coef.push_back(2);
  filt_coef.push_back(1);

  SS.setSmoothCoef(filt_coef);
  computeBoundaries(surface, SS, sorted_boundaries);

  ROS_INFO_STREAM("Boundaries Computed");
  geometry_msgs::PoseArray all_edge_poses;
  for(int i = 0; i < sorted_boundaries.size(); i++)
  {
    if(sorted_boundaries.at(i)->size() < BOUNDARY_THRESHOLD)
      continue;

    geometry_msgs::PoseArray edge_poses;
    visualization_msgs::Marker edge_visualization;

    if(requestEdgePath(sorted_boundaries, i, SS, edge_visualization, edge_poses))
    {
      ProcessPathResult::value_type edge_path_result; // pair<string, geometry_msgs::PoseArray>
      edge_path_result.first = name + "_edge_" + std::to_string(i);

      /*
       * Set the orientation for all edge points to be the orientation of the surface normal
       * This is a hack that should be removed when planar surfaces assumption is dropped
       * The main purpose here is to "smooth" the trajectory of the edges w.r.t. the z axis
      */
      for(auto& p : edge_poses.poses)
        p.orientation = boundary_pose.orientation;

      edge_path_result.second = edge_poses;
      result.paths.push_back(edge_path_result);

      // Add poses to visualization
      all_edge_poses.poses.insert(std::end(all_edge_poses.poses), std::begin(edge_poses.poses),
                                  std::end(edge_poses.poses));

      // Add edge to data coordinator
      data_coordinator_.addEdge(id, edge_path_result.first, edge_poses);
    }
    else
    {
      // Blend path request failed
      ROS_WARN_STREAM("Could not calculate blend path for surface: " << name);
    }
  }

  // Request laser scan paths
  visualization_msgs::Marker scan_visualization;
  geometry_msgs::PoseArray scan_poses;
  if (requestScanPath(filtered_boundaries, boundary_pose, scan_params, scan_visualization, scan_poses))
  {
    ProcessPathResult::value_type scan_path_result;
    scan_path_result.first = name + "_scan";
    scan_path_result.second = scan_poses;
    result.paths.push_back(scan_path_result);

    // Hack for visualization sake
    scan_visualization.header.frame_id = "world_frame";
    scan_visualization.header.stamp = ros::Time::now();
    scan_visualization.id = marker_counter_++;
    scan_visualization.lifetime = ros::Duration(0.0);
    scan_visualization.ns = "scan_paths";
    scan_visualization.pose = boundary_pose;
    scan_visualization.action = visualization_msgs::Marker::ADD;
    scan_visualization.type = visualization_msgs::Marker::LINE_STRIP;
    scan_visualization.scale.x = 0.002;
    std_msgs::ColorRGBA color;
    color.r = 1.0;
    color.b = 0.0;
    color.g = 1.0;
    color.a = 0.5;
    scan_visualization.colors.clear();
    scan_visualization.color = color;

    process_path_results_.scan_visualization_.markers.push_back(scan_visualization);
    data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::scan_pose, id, scan_poses);
  }
  else
  {
    ROS_WARN_STREAM("Could not calculate scan path for surface: " << name);
  }

  return result;
}

godel_surface_detection::TrajectoryLibrary SurfaceBlendingService::generateMotionLibrary(
    const godel_msgs::BlendingPlanParameters& blend_params,
    const godel_msgs::ScanPlanParameters& scan_params)
{
  std::vector<int> selected_ids;
  surface_server_.getSelectedIds(selected_ids);
  std::stringstream ss;
  for (const auto& id : selected_ids)
    ss << id << " ";

  ROS_INFO_STREAM("Selected surface ids " << ss.str());

  // Marker stuff
  process_path_results_.process_visualization_.markers.clear();
  process_path_results_.edge_visualization_.markers.clear();
  process_path_results_.scan_visualization_.markers.clear();
  marker_counter_ = 0;

  godel_surface_detection::TrajectoryLibrary lib;
  geometry_msgs::PoseArray blend_poses, edge_poses;
  for (const auto& id : selected_ids)
  {
    // Generate motion plan
    ProcessPathResult paths = generateProcessPath(id, blend_params, scan_params);

    // Add poses to visualization stack
    for(const auto& vt: paths.paths)
    {
      if(isBlendingPath(vt.first))
        blend_poses.poses.insert(blend_poses.poses.end(), vt.second.poses.begin(), vt.second.poses.end());
      else if(isEdgePath(vt.first))
        edge_poses.poses.insert(edge_poses.poses.end(), vt.second.poses.begin(), vt.second.poses.end());
      else if(isScanPath(vt.first))
        ;
      else
        ROS_ERROR_STREAM("Tried to visualize an unrecognized path type: " << vt.first);
    }

    // Generate trajectory plans from motion plan
    for (std::size_t j = 0; j < paths.paths.size(); ++j)
    {
      ProcessPlanResult plan = generateProcessPlan(paths.paths[j].first, paths.paths[j].second, blend_params,
                                                   scan_params);

      for (std::size_t k = 0; k < plan.plans.size(); ++k)
        lib.get()[plan.plans[k].first] = plan.plans[k].second;
    }

    // Publish poses
    blend_poses.header.frame_id = edge_poses.header.frame_id = "world_frame";
    blend_poses.header.stamp = edge_poses.header.stamp =ros::Time::now();
    blend_visualization_pub_.publish(blend_poses);
    edge_visualization_pub_.publish(edge_poses);
  }

  return lib;
}


ProcessPlanResult
SurfaceBlendingService::generateProcessPlan(const std::string& name,
                                            const geometry_msgs::PoseArray& poses,
                                            const godel_msgs::BlendingPlanParameters& params,
                                            const godel_msgs::ScanPlanParameters& scan_params)
{
  ProcessPlanResult result;

  bool success = false;
  godel_msgs::ProcessPlan process_plan;


  if (isBlendingPath(name))
  {
    godel_msgs::BlendProcessPlanning srv;
    srv.request.path.poses = poses;
    srv.request.params = params;

    success = blend_planning_client_.call(srv);
    process_plan = srv.response.plan;
  }
  else if (isEdgePath(name))
  {
    godel_msgs::BlendProcessPlanning srv;
    srv.request.path.poses = poses;
    srv.request.params = params;

    success = blend_planning_client_.call(srv);
    process_plan = srv.response.plan;
  }
  else
  {
    godel_msgs::KeyenceProcessPlanning srv;
    srv.request.path.poses = poses;
    srv.request.params = scan_params;

    success = keyence_planning_client_.call(srv);
    process_plan = srv.response.plan;
  }

  if (success)
  {
    ProcessPlanResult::value_type plan;
    plan.first = name;
    plan.second = process_plan;
    result.plans.push_back(plan);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to plan for: " << name);
  }

  return result;
}
