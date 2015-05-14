#include <godel_surface_detection/services/surface_blending_service.h>

// Temporary constants for storing blending path `planning parameters
// Will be replaced by loadable, savable parameters
const static std::string BLEND_TRAJECTORY_BAGFILE = "blend_trajectory.bag";
const static std::string BLEND_TRAJECTORY_GROUP_NAME = "manipulator_tcp";
const static std::string BLEND_TRAJECTORY_TOOL_FRAME = "tcp_frame";
const static std::string BLEND_TRAJECTORY_WORLD_FRAME = "world_frame";
const static double BLEND_TRAJECTORY_ANGLE_DISC = M_PI/10.0;
const static double BLEND_TRAJECTORY_INTERPOINT_DELAY = 0.5;

// Temporary constants for storing scan path planning parameters
// Will be replaced by loadable, savable parameters
const static std::string SCAN_TRAJECTORY_BAGFILE = "scan_trajectory.bag";
const static std::string SCAN_TRAJECTORY_GROUP_NAME = "manipulator_keyence";
const static std::string SCAN_TRAJECTORY_TOOL_FRAME = "keyence_tcp_frame";
const static std::string SCAN_TRAJECTORY_WORLD_FRAME = "world_frame";
const static double SCAN_TRAJECTORY_ANGLE_DISC = 0.2;
const static double SCAN_TRAJECTORY_INTERPOINT_DELAY = 0.5;

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

bool SurfaceBlendingService::requestBlendPath(const godel_process_path::PolygonBoundaryCollection& boundaries,
                      const geometry_msgs::Pose& boundary_pose,
                      const godel_msgs::BlendingPlanParameters& params,
                      visualization_msgs::Marker& path)
{
  godel_process_path_generation::VisualizeBlendingPlan srv;
  srv.request.params = params;
  godel_process_path::utils::translations::godelToGeometryMsgs(srv.request.surface.boundaries, boundaries);
  tf::poseTFToMsg(tf::Transform::getIdentity(), srv.request.surface.pose);

  if (!visualize_process_path_client_.call(srv))
  {
    return false;
  }

  // blend process path calculations suceeded. Save data into results.
  path  = srv.response.path;
  path.ns = PATH_NAMESPACE;
  path.pose = boundary_pose;

  return true;
}

bool SurfaceBlendingService::requestScanPath(const godel_process_path::PolygonBoundaryCollection& boundaries,
                     const geometry_msgs::Pose& boundary_pose,
                     visualization_msgs::Marker& path)
{
  using namespace godel_process_path;
  using godel_process_path::utils::translations::godelToVisualizationMsgs;

  if (boundaries.empty()) return false;

  godel_surface_detection::ProfilimeterScanParams scan_params;
  scan_params.width_ = 0.02;
  scan_params.overlap_ = 0.0;

  PolygonBoundary scan = godel_surface_detection::generateProfilimeterScanPath(boundaries.front(), scan_params);
  utils::translations::godelToVisualizationMsgs(path, scan, std_msgs::ColorRGBA());
  path.pose = boundary_pose;

  return true;
}

ProcessPathResult SurfaceBlendingService::generateProcessPath(const std::string& name, 
                                      const pcl::PolygonMesh& mesh, 
                                      const godel_msgs::BlendingPlanParameters& params)
{
  using godel_process_path::PolygonBoundaryCollection;
  using godel_process_path::PolygonBoundary;
  process_path_results_.process_paths_.markers.clear();

  ProcessPathResult result;

  // Calculate boundaries for a surface
  if (!mesh_importer_.calculateSimpleBoundary(mesh))
  {
    ROS_WARN_STREAM("Could not calculate boundary for mesh associated with name: " << name);
    return result;
  }

  // Read & filter boundaries that are ill-formed or too small 
  PolygonBoundaryCollection filtered_boundaries = filterPolygonBoundaries(mesh_importer_.getBoundaries(), params);
  // Read pose
  geometry_msgs::Pose boundary_pose;
  mesh_importer_.getPose(boundary_pose);

  // Send request to blend path generation service
  visualization_msgs::Marker blend_path;
  if (requestBlendPath(filtered_boundaries, boundary_pose, params, blend_path))
  {
    ProcessPathResult::value_type blend_path_result; // pair<string, viz_msgs::Marker>
    blend_path_result.first = name + "_blend";
    blend_path_result.second = blend_path;
    result.paths.push_back(blend_path_result);

    // Hack for visualization sake
    static unsigned marker_counter = 0;
    blend_path.header.frame_id = "world_frame";
    blend_path.id = marker_counter++;
    blend_path.lifetime = ros::Duration(0);
    blend_path.ns = PATH_NAMESPACE;
    blend_path.pose = boundary_pose;
    process_path_results_.process_paths_.markers.push_back(blend_path);
  }
  else
  {
    // Blend path request failed
    ROS_WARN_STREAM("Could not calculate blend path for surface: " << name);
  }

  // Request laser scan paths
  visualization_msgs::Marker scan_path;
  if (requestScanPath(filtered_boundaries, boundary_pose, scan_path))
  {
    ProcessPathResult::value_type scan_path_result;
    scan_path_result.first = name + "_scan";
    scan_path_result.second = scan_path;
    result.paths.push_back(scan_path_result);
  }
  else
  {
    ROS_WARN_STREAM("Could not calculate scan path for surface: " << name);
  }

  return result;
}

godel_surface_detection::TrajectoryLibrary SurfaceBlendingService::generateMotionLibrary(const godel_msgs::BlendingPlanParameters& params)
{
  std::vector<pcl::PolygonMesh> meshes;
  surface_server_.get_selected_surfaces(meshes);
  std::vector<std::string> names;
  surface_server_.get_selected_list(names);

  ROS_ASSERT(names.size() == meshes.size());

  godel_surface_detection::TrajectoryLibrary lib;
  for (std::size_t i = 0; i < meshes.size(); ++i)
  {
    ProcessPathResult paths = generateProcessPath(names[i], meshes[i], params);
    for (std::size_t j = 0; j < paths.paths.size(); ++j)
    {
      ProcessPlanResult plan = generateProcessPlan(paths.paths[j].first, paths.paths[j].second);
      for (std::size_t k = 0; k < plan.plans.size(); ++k)
      {
        lib.get()[plan.plans[k].first] = plan.plans[k].second;
      }
    }
  }
  return lib;
}

static inline bool isBlendingPath(const std::string& name)
{
  const static std::string suffix ("_blend");
  if (name.size() < suffix.size()) return false;
  return name.find(suffix, name.size()-suffix.length()) != std::string::npos;
}

static godel_msgs::TrajectoryPlanningRequest getBlendTrajectoryRequest()
{
  godel_msgs::TrajectoryPlanningRequest request;
  request.group_name = BLEND_TRAJECTORY_GROUP_NAME;
  request.tool_frame = BLEND_TRAJECTORY_TOOL_FRAME;
  request.world_frame = BLEND_TRAJECTORY_WORLD_FRAME;
  request.angle_discretization = BLEND_TRAJECTORY_ANGLE_DISC;
  request.interpoint_delay = BLEND_TRAJECTORY_INTERPOINT_DELAY;
  request.is_blending_path = true;
  request.plan_from_current_position = true;
  return request;
}

static godel_msgs::TrajectoryPlanningRequest getScanTrajectoryRequest()
{
  godel_msgs::TrajectoryPlanningRequest request;
  request.group_name = SCAN_TRAJECTORY_GROUP_NAME;
  request.tool_frame = SCAN_TRAJECTORY_TOOL_FRAME;
  request.world_frame = SCAN_TRAJECTORY_WORLD_FRAME;
  request.angle_discretization = SCAN_TRAJECTORY_ANGLE_DISC;
  request.interpoint_delay = SCAN_TRAJECTORY_INTERPOINT_DELAY;
  request.is_blending_path = false;
  request.plan_from_current_position = false;
  return request;
}

ProcessPlanResult SurfaceBlendingService::generateProcessPlan(const std::string& name, 
                                      const visualization_msgs::Marker& path)
{
  ProcessPlanResult result;

  godel_msgs::TrajectoryPlanningRequest req = isBlendingPath(name) ? getBlendTrajectoryRequest() : getScanTrajectoryRequest();
  req.path.reference = path.pose;
  req.path.points = path.points;
    
  ROS_INFO_STREAM("Calling trajectory planner for process: " << name);
  godel_msgs::TrajectoryPlanningResponse res;
  if (!trajectory_planner_client_.call(req, res))
  {
    ROS_WARN_STREAM("Failed to plan for path: " << name);
    return result;
  }

  ProcessPlanResult::value_type plan;
  plan.first = name;
  plan.second = res.trajectory;
  result.plans.push_back(plan);
  
  return result;
}
