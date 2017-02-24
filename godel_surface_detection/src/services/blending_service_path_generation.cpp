#include <detection/surface_detection.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pluginlib/class_loader.h>
#include <ros/node_handle.h>
#include <services/surface_blending_service.h>
#include <segmentation/surface_segmentation.h>
#include <eigen_conversions/eigen_msg.h>
#include <path_planning_plugins_base/path_planning_base.h>

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

// Edge Processing constants
const static double SEGMENTATION_SEARCH_RADIUS = 0.03; // 3cm
const static int BOUNDARY_THRESHOLD = 10;

// Variables to select path type
const static int PATH_TYPE_BLENDING = 0;
const static int PATH_TYPE_SCAN = 1;
const static int PATH_TYPE_EDGE = 2;

const static std::string SURFACE_DESIGNATION = "surface_marker_server_";

const static std::string PARAM_BASE = "/process_planning_params/";
const static std::string SCAN_PARAM_BASE = "scan_params/";
const static std::string BLEND_PARAM_BASE = "blend_params/";
const static std::string SPINDLE_SPEED_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "spindle_speed";
const static std::string APPROACH_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "approach_speed";
const static std::string BLENDING_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "blending_speed";
const static std::string RETRACT_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "retract_speed";
const static std::string TRAVERSE_SPD_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "traverse_speed";
const static std::string APPROACH_DISTANCE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "approach_distance";
const static std::string QUALITY_METRIC_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "quality_metric";
const static std::string WINDOW_WIDTH_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "window_width";
const static std::string MIN_QA_VALUE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "min_qa_value";
const static std::string MAX_QA_VALUE_PARAM = PARAM_BASE + SCAN_PARAM_BASE + "max_qa_value";


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


bool SurfaceBlendingService::generateEdgePath(godel_surface_detection::detection::CloudRGB::Ptr surface,
                                              std::vector<geometry_msgs::PoseArray>& result)
{
  // Send request to edge path generation service
  std::vector<pcl::IndicesPtr> sorted_boundaries;

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

  for(int i = 0; i < sorted_boundaries.size(); i++)
  {
    if(sorted_boundaries.at(i)->size() < BOUNDARY_THRESHOLD)
      continue;

    geometry_msgs::PoseArray edge_poses;
    geometry_msgs::Pose geo_pose;
    std::vector<Eigen::Matrix4d, Eigen::aligned_allocator<Eigen::Matrix4d>> poses;

    // Get boundary trajectory and trim last two poses (last poses are susceptible to large velocity changes)
    SS.getBoundaryTrajectory(sorted_boundaries, i, poses);
    poses.resize(poses.size() - 2);

    // Convert eigen poses to geometry poses for messaging and visualization
    for(const auto& p : poses)
    {
      Eigen::Affine3d pose(p.matrix());
      tf::poseEigenToMsg(pose, geo_pose);
      edge_poses.poses.push_back(geo_pose);
    }

    result.push_back(edge_poses);
  }
  return result.size() > 0;
}


bool
SurfaceBlendingService::generateProcessPath(const int& id,
                                            ProcessPathResult& result)
{
  using godel_surface_detection::detection::CloudRGB;

  std::string name;
  pcl::PolygonMesh mesh;
  CloudRGB::Ptr surface_ptr (new CloudRGB);

  data_coordinator_.getSurfaceName(id, name);
  data_coordinator_.getSurfaceMesh(id, mesh);
  data_coordinator_.getCloud(godel_surface_detection::data::CloudTypes::surface_cloud, id, *surface_ptr);
  return generateProcessPath(id, name, mesh, surface_ptr, result);
}


bool
SurfaceBlendingService::generateProcessPath(const int& id,
                                            const std::string& name,
                                            const pcl::PolygonMesh& mesh,
                                            godel_surface_detection::detection::CloudRGB::Ptr surface,
                                            ProcessPathResult& result)
{
  std::vector<geometry_msgs::PoseArray> blend_result, edge_result, scan_result;

  pluginlib::ClassLoader<path_planning_plugins_base::PathPlanningBase> poly_loader("path_planning_plugins_base",
                                                                           "path_planning_plugins_base::PathPlanningBase");
  boost::shared_ptr<path_planning_plugins_base::PathPlanningBase> path_planner;

  try
  {
    path_planner = poly_loader.createInstance(getBlendToolPlanningPluginName());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The blend plugin failed to load for some reason. Error: %s", ex.what());
    return false;
  }

  path_planner->init(mesh);
  if(!path_planner->generatePath(blend_result))
  {
    ROS_ERROR_STREAM("Could not plan blend path");
    return false;
  }

  try
  {
    path_planner = poly_loader.createInstance(getScanToolPlanningPluginName());
  }
  catch(pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("The scan plugin failed to load for some reason. Error: %s", ex.what());
    return false;
  }

  path_planner->init(mesh);
  if(!path_planner->generatePath(scan_result))
  {
    ROS_ERROR_STREAM("Could not plan scan path");
    return false;
  }

  generateEdgePath(surface, edge_result);

  ProcessPathResult::value_type vt;

  vt.first = name + "_blend";
  vt.second = blend_result;
  result.paths.push_back(vt);
  data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::blend_pose, id, vt.second);

  vt.first = name + "_scan";
  vt.second = scan_result;
  result.paths.push_back(vt);
  data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::scan_pose, id, vt.second);

  int i = 0;
  for(const auto& pose_array : edge_result)
  {
    vt.first = name + "_edge_" + std::to_string(i++);
    std::vector<geometry_msgs::PoseArray> temp;
    temp.push_back(pose_array);
    vt.second = std::move(temp);
    result.paths.push_back(vt);
    data_coordinator_.addEdge(id, vt.first, pose_array);
  }
  return result.paths.size() > 0;
}

godel_surface_detection::TrajectoryLibrary SurfaceBlendingService::generateMotionLibrary(
    const godel_msgs::PathPlanningParameters& params)
{
  std::vector<int> selected_ids;
  surface_server_.getSelectedIds(selected_ids);

  godel_surface_detection::TrajectoryLibrary lib;
  // Clear previous results
  process_path_results_.blend_poses_.clear();
  process_path_results_.edge_poses_.clear();
  process_path_results_.scan_poses_.clear();

  for (const auto& id : selected_ids)
  {
    // Generate motion plan
    ProcessPathResult paths;
    generateProcessPath(id, paths);

    // If planning failed entirely, skip to next
    if(paths.paths.size() == 0)
      continue;

    // Add new path to result
    for(const auto& vt: paths.paths)
    {
      if(isBlendingPath(vt.first))
        process_path_results_.blend_poses_.push_back(vt.second);

      else if(isEdgePath(vt.first))
        process_path_results_.edge_poses_.push_back(vt.second.front());

      else if(isScanPath(vt.first))
        process_path_results_.scan_poses_.push_back(vt.second);

      else
        ROS_ERROR_STREAM("Tried to process an unrecognized path type: " << vt.first);
    }

    ros::NodeHandle nh;

    godel_msgs::BlendingPlanParameters blend_params;
    blend_params.margin = params.margin;
    blend_params.overlap = params.overlap;
    blend_params.tool_radius = params.tool_radius;
    blend_params.discretization = params.discretization;
    blend_params.safe_traverse_height = params.traverse_height;
    nh.getParam(SPINDLE_SPEED_PARAM, blend_params.spindle_speed);
    nh.getParam(APPROACH_SPD_PARAM, blend_params.approach_spd);
    nh.getParam(BLENDING_SPD_PARAM, blend_params.blending_spd);
    nh.getParam(RETRACT_SPD_PARAM, blend_params.retract_spd);
    nh.getParam(TRAVERSE_SPD_PARAM, blend_params.traverse_spd);

    godel_msgs::ScanPlanParameters scan_params;
    scan_params.scan_width = params.scan_width;
    scan_params.margin = params.margin;
    scan_params.overlap = params.overlap;
    scan_params.scan_width = params.scan_width;
    nh.getParam(APPROACH_DISTANCE_PARAM, scan_params.approach_distance);
    nh.getParam(TRAVERSE_SPD_PARAM, scan_params.traverse_spd);
    nh.getParam(QUALITY_METRIC_PARAM, scan_params.quality_metric);
    nh.getParam(WINDOW_WIDTH_PARAM, scan_params.window_width);
    nh.getParam(MIN_QA_VALUE_PARAM, scan_params.min_qa_value);
    nh.getParam(MAX_QA_VALUE_PARAM, scan_params.min_qa_value);


    // Generate trajectory plans from motion plan
    for (std::size_t j = 0; j < paths.paths.size(); ++j)
    {
      ProcessPlanResult plan = generateProcessPlan(paths.paths[j].first, paths.paths[j].second, blend_params,
                                                   scan_params);

      for (std::size_t k = 0; k < plan.plans.size(); ++k)
        lib.get()[plan.plans[k].first] = plan.plans[k].second;
    }
  }

  return lib;
}


ProcessPlanResult
SurfaceBlendingService::generateProcessPlan(const std::string& name,
                                            const std::vector<geometry_msgs::PoseArray>& poses,
                                            const godel_msgs::BlendingPlanParameters& params,
                                            const godel_msgs::ScanPlanParameters& scan_params)
{
  ProcessPlanResult result;

  bool success = false;
  godel_msgs::ProcessPlan process_plan;


  if (isBlendingPath(name))
  {
    godel_msgs::BlendProcessPlanning srv;
    srv.request.path.segments = poses;
    srv.request.params = params;

    success = blend_planning_client_.call(srv);
    process_plan = srv.response.plan;
  }
  else if (isEdgePath(name))
  {
    godel_msgs::BlendProcessPlanning srv;
    srv.request.path.segments = poses;
    srv.request.params = params;

    success = blend_planning_client_.call(srv);
    process_plan = srv.response.plan;
  }
  else
  {
    godel_msgs::KeyenceProcessPlanning srv;
    srv.request.path.segments = poses;
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
