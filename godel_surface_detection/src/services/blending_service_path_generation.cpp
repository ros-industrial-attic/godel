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

#include <pcl/segmentation/extract_clusters.h>
#include <swri_profiler/profiler.h>

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
const static std::string Z_ADJUST_PARAM = PARAM_BASE + BLEND_PARAM_BASE + "z_adjust";

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
  SWRI_PROFILE("gen-edge-path");
  // Send request to edge path generation service
  std::vector<pcl::IndicesPtr> sorted_boundaries;

  // Compute the boundary
  SurfaceSegmentation SS(surface);

  SS.setSearchRadius(SEGMENTATION_SEARCH_RADIUS);
  computeBoundaries(surface, SS, sorted_boundaries);

  ROS_INFO_STREAM("Boundaries Computed = "<<sorted_boundaries.size());

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

  ROS_INFO_COND((result.size() > 0),"Finished generating edge paths for %i boundaries",int(result.size()));
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

static bool generateToolPaths(const godel_msgs::PathPlanningParameters& params,
                              const pcl::PolygonMesh& mesh,
                              const std::string& plugin_name,
                              std::vector<geometry_msgs::PoseArray>& result)
{
  pluginlib::ClassLoader<path_planning_plugins_base::PathPlanningBase>
      loader("path_planning_plugins_base", "path_planning_plugins_base::PathPlanningBase");
  auto planner = loader.createInstance(plugin_name);
  planner->init(mesh);
  return planner->generatePath(result);
}

bool SurfaceBlendingService::generateBlendPath(const godel_msgs::PathPlanningParameters &params,
                                               const pcl::PolygonMesh &mesh, std::vector<geometry_msgs::PoseArray> &result)
{
  SWRI_PROFILE("gen-blend-path");
  try
  {
    if (!generateToolPaths(params, mesh, getBlendToolPlanningPluginName(), result))
    {
      ROS_ERROR("Failed to generate tool paths for blend process");
      return false;
    }
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("Blending tool planning plugin loading failed with error: '%s'", ex.what());
    return false;
  }
  return true;
}

bool SurfaceBlendingService::generateScanPath(const godel_msgs::PathPlanningParameters &params, const pcl::PolygonMesh &mesh,
                                              std::vector<geometry_msgs::PoseArray> &result)
{
  SWRI_PROFILE("gen-scan-path");
  try
  {
    if (!generateToolPaths(params, mesh, getScanToolPlanningPluginName(), result))
    {
      ROS_ERROR("Failed to generate tool paths for scan process");
      return false;
    }
  }
  catch(const pluginlib::PluginlibException& ex)
  {
    ROS_ERROR("Scan tool planning plugin loading failed with error: '%s'", ex.what());
    return false;
  }
  return true;
}

bool
SurfaceBlendingService::generateProcessPath(const int& id,
                                            const std::string& name,
                                            const pcl::PolygonMesh& mesh,
                                            godel_surface_detection::detection::CloudRGB::Ptr surface,
                                            ProcessPathResult& result)
{
  SWRI_PROFILE("tool-planning");
  std::vector<geometry_msgs::PoseArray> blend_result, edge_result, scan_result;

  // Step 1: Generate Blending Paths
  godel_msgs::PathPlanningParameters params;
  if (!generateBlendPath(params, mesh, blend_result))
  {
    process_planning_feedback_.last_completed = "Failed to generate blend path for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);
  }
  else
  {
    process_planning_feedback_.last_completed = "Generated blend path for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);

    // Add the successful blend path to the output
    ProcessPathResult::value_type vt;
    vt.first = name + "_blend";
    vt.second = blend_result;
    result.paths.push_back(vt);
    data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::blend_pose, id, vt.second);
  }

  // Step 2: Generate Laser Scan Paths
  if (!generateScanPath(params, mesh, scan_result))
  {
    process_planning_feedback_.last_completed = "Failed to generate scan path for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);
  }
  else
  {
    process_planning_feedback_.last_completed = "Generated scan path for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);

    // Add the successful scan path to the output
    ProcessPathResult::value_type vt;
    vt.first = name + "_scan";
    vt.second = scan_result;
    result.paths.push_back(vt);
    data_coordinator_.setPoses(godel_surface_detection::data::PoseTypes::scan_pose, id, vt.second);
  }

  // Step 3: Generate Edge Paths for the given surface
  if (!generateEdgePath(surface, edge_result))
  {
    process_planning_feedback_.last_completed = "Failed to generate generate edge path(s) for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);
  }
  else
  {
    process_planning_feedback_.last_completed = "Generated edge path(s) for surface " + name;
    process_planning_server_.publishFeedback(process_planning_feedback_);

    // Add the edge paths to the results
    ProcessPathResult::value_type vt;
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
  }

  return result.paths.size() > 0;
}

godel_surface_detection::TrajectoryLibrary SurfaceBlendingService::generateMotionLibrary(
    const godel_msgs::PathPlanningParameters& params)
{
  std::vector<int> selected_ids;
  surface_server_.getSelectedIds(selected_ids);
  return generateMotionLibrary(params, selected_ids);
}

// static helper function
static std::vector<pcl::PointCloud<pcl::PointXYZRGB>>
computeQAClusters(const pcl::PointCloud<pcl::PointXYZRGB>& macro_surface,
                  const pcl::PointCloud<pcl::PointXYZRGB>& laser_surface,
                  const cat_laser_scan_qa::TorchCutQAResult& qa_result)
{
  // Let's extract the high points
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr high_points (new pcl::PointCloud<pcl::PointXYZRGB>());
  pcl::copyPointCloud(laser_surface, qa_result.high_point_indices.indices, *high_points);

  // 'Dilate' the points by finding neighbors of the high points
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud(macro_surface.makeShared());

  std::set<int> indices;
  for (const auto& pt : *high_points)
  {
    std::vector<int> local_indices;
    std::vector<float> distances;
    tree->radiusSearch(pt, 0.01, local_indices, distances);
    indices.insert(local_indices.begin(), local_indices.end());
  }

  ROS_ERROR_STREAM("input cloud had " << high_points->size() << "points and after dilation we have " << indices.size());

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr dilated_cloud (new pcl::PointCloud<pcl::PointXYZRGB>());
  std::vector<int> dilated_indices;
  dilated_indices.insert(dilated_indices.end(), indices.begin(), indices.end());
  pcl::copyPointCloud(macro_surface, dilated_indices, *dilated_cloud);


  // Cluster the high points
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree2->setInputCloud(dilated_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance(0.01);
  ec.setMinClusterSize(10);
//  ec.setMaxClusterSize(25000);
  ec.setSearchMethod(tree2);
  ec.setInputCloud(dilated_cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  ec.extract(cluster_indices);

  ROS_ERROR_STREAM("After clustering we have " << cluster_indices.size() << " clusters");

  // Assemble results
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>> clusters (cluster_indices.size());
  for (std::size_t i = 0; i < cluster_indices.size(); ++i)
  {
    ROS_ERROR_STREAM("Cluster " << i << " has " << cluster_indices[i].indices.size() << " points");
    pcl::copyPointCloud(*dilated_cloud, cluster_indices[i].indices, clusters[i]);
  }

  return clusters;
}

godel_surface_detection::TrajectoryLibrary
SurfaceBlendingService::generateQAMotionLibrary(const godel_msgs::PathPlanningParameters &params)
{
  // Define return value library which will get populated for each surface
  godel_surface_detection::TrajectoryLibrary lib;

  // Identify any outstanding QA 'jobs'
  for (const auto& q : qa_server_)
  {
    // For each active job...
    const int key = q.first;
    const godel_qa_server::QAJob& job = q.second;
    ROS_ERROR_STREAM("CONSIDERING QA FOR SURFACE " << key);

    // Examine the QA point cloud and pull out "patches" that need to be re-processed; each patch is a euclidean
    // cluster
    pcl::PointCloud<pcl::PointXYZRGB> surface_cloud, laser_cloud;
    data_coordinator_.getCloud(godel_surface_detection::data::surface_cloud, key, surface_cloud);
    data_coordinator_.getCloud(godel_surface_detection::data::laser_cloud, key, laser_cloud);
    std::string surface_name;
    data_coordinator_.getSurfaceName(key, surface_name);

    auto clusters = computeQAClusters(surface_cloud, laser_cloud, job.iterations().back().qa_result);

    // For each cluster, lets create a new record in the data coordinator whilst building a list of new surfaces
    std::vector<int> new_surface_ids;

    for (std::size_t i = 0; i < clusters.size(); ++i)
    {
      auto new_id = data_coordinator_.addRecord(surface_cloud, clusters[i]);
      ROS_ERROR_STREAM("Creating a new qa surface w/ id = " << new_id);
      const std::string new_surface_name = surface_name + "_qa" + std::to_string(i);
      data_coordinator_.setSurfaceName(new_id, new_surface_name);

      pcl::PolygonMesh mesh;
      surface_detection_.meshCloud(clusters[i], mesh);
      data_coordinator_.setSurfaceMesh(new_id, mesh);

      if (mesh.polygons.size() > 0)
      {
        new_surface_ids.push_back(new_id);
      }
      else
      {
        ROS_ERROR_STREAM("Mesh for surface " << key << " cluster " << i << " new id = " << new_id << " has no verts");
      }
    }

    // Generate a new motion library with plans for each surface patch
    auto patch_plans = generateMotionLibrary(params, new_surface_ids);

    // Merge the motion library with the overall results
    lib.merge(patch_plans);
  }

  return lib;
}

godel_surface_detection::TrajectoryLibrary
SurfaceBlendingService::generateMotionLibrary(const godel_msgs::PathPlanningParameters &params,
                                              const std::vector<int> &selected_surface_ids)
{
  SWRI_PROFILE("generate-motion-library");
  godel_surface_detection::TrajectoryLibrary lib;
  // Clear previous results
  process_path_results_.blend_poses_.clear();
  process_path_results_.edge_poses_.clear();
  process_path_results_.scan_poses_.clear();

  for (const auto& id : selected_surface_ids)
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
    nh.getParam(Z_ADJUST_PARAM, blend_params.z_adjust);

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
//    nh.getParam(Z_ADJUST_PARAM, scan_params.z_adjust);
    scan_params.z_adjust = 0.0; // Until we fix these parameters and do not share them among the
                                // different processes, I'm only applying this to blend paths.

    // Generate trajectory plans from motion plan
    {
      SWRI_PROFILE("motion-planning");
      for (std::size_t j = 0; j < paths.paths.size(); ++j)
      {
        ProcessPlanResult plan = generateProcessPlan(paths.paths[j].first, paths.paths[j].second, blend_params,
                                                     scan_params);

        for (std::size_t k = 0; k < plan.plans.size(); ++k)
        {
          plan.plans[k].second.id = id;
          lib.get()[plan.plans[k].first] = plan.plans[k].second;
        }
      }
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
