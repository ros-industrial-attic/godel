#include <godel_surface_detection/services/surface_blending_service.h>

#include <godel_msgs/TrajectoryExecution.h>

// Process Execution
#include <godel_msgs/BlendProcessExecution.h>
#include <godel_msgs/KeyenceProcessExecution.h>
// Process Planning
#include <godel_msgs/BlendProcessPlanning.h>
#include <godel_msgs/KeyenceProcessPlanning.h>

#include <godel_param_helpers/godel_param_helpers.h>

// topics and services
const static std::string TRAJECTORY_PLANNING_SERVICE = "trajectory_planner";
const static std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const static std::string SURFACE_BLENDING_PARAMETERS_SERVICE = "surface_blending_parameters";
const static std::string SELECT_SURFACE_SERVICE = "select_surface";
const static std::string PROCESS_PATH_SERVICE = "process_path";
const static std::string VISUALIZE_BLENDING_PATH_SERVICE = "visualize_path_generator";
const static std::string RENAME_SURFACE_SERVICE = "rename_surface";
const static std::string PATH_EXECUTION_SERVICE = "path_execution";
const static std::string GET_MOTION_PLANS_SERVICE = "get_available_motion_plans";
const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";
const static std::string LOAD_SAVE_MOTION_PLAN_SERVICE = "load_save_motion_plan";

const static std::string BLEND_PROCESS_EXECUTION_SERVICE = "blend_process_execution";
const static std::string SCAN_PROCESS_EXECUTION_SERVICE = "scan_process_execution";
const static std::string BLEND_PROCESS_PLANNING_SERVICE = "blend_process_planning";
const static std::string SCAN_PROCESS_PLANNING_SERVICE = "keyence_process_planning";

const static std::string TOOL_PATH_PREVIEW_TOPIC = "tool_path_preview";
const static std::string SELECTED_SURFACES_CHANGED_TOPIC = "selected_surfaces_changed";
const static std::string ROBOT_SCAN_PATH_PREVIEW_TOPIC = "robot_scan_path_preview";
const static std::string PUBLISH_REGION_POINT_CLOUD = "publish_region_point_cloud";
const static std::string REGION_POINT_CLOUD_TOPIC = "region_colored_cloud";

//  tool visual properties
const static float TOOL_DIA = .050;
const static float TOOL_THK = .005;
const static float TOOL_SHAFT_DIA = .006;
const static float TOOL_SHAFT_LEN = .045;
const static std::string TOOL_FRAME_ID = "process_tool";

// Default filepaths and namespaces for caching stored parameters
const static std::string BLEND_PARAMS_FILE = "godel_blending_parameters.msg";

const static std::string SCAN_PARAMS_FILE = "godel_scan_parameters.msg";

const static std::string ROBOT_SCAN_PARAMS_FILE = "godel_robot_scan_parameters.msg";

const static std::string SURFACE_DETECTION_PARAMS_FILE = "godel_surface_detection_parameters.msg";

SurfaceBlendingService::SurfaceBlendingService()
    : mesh_importer_(true) /*True-turn on verbose messages*/
      ,
      publish_region_point_cloud_(false)
{
}

bool SurfaceBlendingService::init()
{
  using namespace godel_surface_detection;

  ros::NodeHandle ph("~");

  // loading parameters
  ph.getParam(PUBLISH_REGION_POINT_CLOUD, publish_region_point_cloud_);
  // Load the 'prefix' that will be combined with parameters msg base names
  // to save to disk
  ph.param<std::string>("param_cache_prefix", param_cache_prefix_, "");

  if (!this->load_blend_parameters(param_cache_prefix_ + BLEND_PARAMS_FILE))
  {
    ROS_WARN("Unable to load blending process parameters.");
  }
  if (!this->load_scan_parameters(param_cache_prefix_ + SCAN_PARAMS_FILE))
  {
    ROS_WARN("Unable to load scan process parameters.");
  }
  if (!robot_scan_.load_parameters(param_cache_prefix_ + ROBOT_SCAN_PARAMS_FILE))
  {
    ROS_WARN("Unable to load robot macro scan parameters.");
  }
  if (!surface_detection_.load_parameters(param_cache_prefix_ + SURFACE_DETECTION_PARAMS_FILE))
  {
    ROS_WARN("Unable to load surface detection parameters.");
  }

  // save default parameters
  default_robot_scan_params__ = robot_scan_.params_;
  default_surf_detection_params_ = surface_detection_.params_;
  default_blending_plan_params_ = blending_plan_params_;
  default_scan_params_ = scan_plan_params_;

  if (surface_detection_.init() && robot_scan_.init() && surface_server_.init())
  {
    // adding callbacks
    scan::RobotScan::ScanCallback cb =
        boost::bind(&detection::SurfaceDetection::add_cloud, &surface_detection_, _1);
    robot_scan_.add_scan_callback(cb);
    ROS_INFO_STREAM("Surface detection service initialization succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Surface detection service had an initialization error");
  }

  // start server
  interactive::InteractiveSurfaceServer::SelectionCallback f =
      boost::bind(&SurfaceBlendingService::publish_selected_surfaces_changed, this);
  surface_server_.add_selection_callback(f);

  // initializing ros interface
  ros::NodeHandle nh;

  // service clients
  visualize_process_path_client_ =
      nh.serviceClient<godel_process_path_generation::VisualizeBlendingPlan>(
          VISUALIZE_BLENDING_PATH_SERVICE);

  // Process Execution Parameters
  blend_process_client_ =
      nh.serviceClient<godel_msgs::BlendProcessExecution>(BLEND_PROCESS_EXECUTION_SERVICE);
  scan_process_client_ =
      nh.serviceClient<godel_msgs::KeyenceProcessExecution>(SCAN_PROCESS_EXECUTION_SERVICE);

  blend_planning_client_ =
      nh.serviceClient<godel_msgs::BlendProcessPlanning>(BLEND_PROCESS_PLANNING_SERVICE);
  keyence_planning_client_ =
      nh.serviceClient<godel_msgs::KeyenceProcessPlanning>(SCAN_PROCESS_PLANNING_SERVICE);

  // service servers
  surf_blend_parameters_server_ =
      nh.advertiseService(SURFACE_BLENDING_PARAMETERS_SERVICE,
                          &SurfaceBlendingService::surface_blend_parameters_server_callback, this);

  surface_detect_server_ = nh.advertiseService(
      SURFACE_DETECTION_SERVICE, &SurfaceBlendingService::surface_detection_server_callback, this);

  select_surface_server_ = nh.advertiseService(
      SELECT_SURFACE_SERVICE, &SurfaceBlendingService::select_surface_server_callback, this);

  process_path_server_ = nh.advertiseService(
      PROCESS_PATH_SERVICE, &SurfaceBlendingService::process_path_server_callback, this);

  get_motion_plans_server_ = nh.advertiseService(
      GET_MOTION_PLANS_SERVICE, &SurfaceBlendingService::getMotionPlansCallback, this);

  select_motion_plan_server_ = nh.advertiseService(
      SELECT_MOTION_PLAN_SERVICE, &SurfaceBlendingService::selectMotionPlanCallback, this);

  load_save_motion_plan_server_ = nh.advertiseService(
      LOAD_SAVE_MOTION_PLAN_SERVICE, &SurfaceBlendingService::loadSaveMotionPlanCallback, this);

  rename_suface_server_ = nh.advertiseService(RENAME_SURFACE_SERVICE,
                                              &SurfaceBlendingService::renameSurfaceCallback, this);

  // publishers
  selected_surf_changed_pub_ =
      nh.advertise<godel_msgs::SelectedSurfacesChanged>(SELECTED_SURFACES_CHANGED_TOPIC, 1);

  point_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>(REGION_POINT_CLOUD_TOPIC, 1);

  tool_path_markers_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>(TOOL_PATH_PREVIEW_TOPIC, 1, true);

  return true;
}

void SurfaceBlendingService::run()
{
  surface_server_.run();

  ros::Duration loop_duration(1.0f);
  while (ros::ok())
  {
    if (publish_region_point_cloud_ && !region_cloud_msg_.data.empty())
    {
      point_cloud_pub_.publish(region_cloud_msg_);
    }

    loop_duration.sleep();
  }
}

// Blending Parameters
bool SurfaceBlendingService::load_blend_parameters(const std::string& filename)
{
  using godel_param_helpers::loadParam;
  using godel_param_helpers::loadBoolParam;

  if (godel_param_helpers::fromFile(filename, blending_plan_params_))
  {
    return true;
  }
  // otherwise default to the parameter server
  ros::NodeHandle nh("~/blending_plan");
  return loadParam(nh, "tool_radius", blending_plan_params_.tool_radius) &&
         loadParam(nh, "margin", blending_plan_params_.margin) &&
         loadParam(nh, "overlap", blending_plan_params_.overlap) &&
         loadParam(nh, "approach_spd", blending_plan_params_.approach_spd) &&
         loadParam(nh, "blending_spd", blending_plan_params_.blending_spd) &&
         loadParam(nh, "retract_spd", blending_plan_params_.retract_spd) &&
         loadParam(nh, "traverse_spd", blending_plan_params_.traverse_spd) &&
         loadParam(nh, "discretization", blending_plan_params_.discretization) &&
         loadParam(nh, "safe_traverse_height", blending_plan_params_.safe_traverse_height) &&
         loadParam(nh, "min_boundary_length", blending_plan_params_.min_boundary_length);
}

void SurfaceBlendingService::save_blend_parameters(const std::string& filename)
{
  if (!godel_param_helpers::toFile(filename, blending_plan_params_))
  {
    ROS_WARN_STREAM("Unable to save blending-plan parameters to: " << filename);
  }
}

// Profilimeter parameters
bool SurfaceBlendingService::load_scan_parameters(const std::string& filename)
{
  using godel_param_helpers::loadParam;
  using godel_param_helpers::loadBoolParam;

  if (godel_param_helpers::fromFile(filename, scan_plan_params_))
  {
    return true;
  }

  // otherwise we load default parameters from the param server
  ros::NodeHandle nh("~/scan_plan");
  return loadParam(nh, "traverse_speed", scan_plan_params_.traverse_spd) &&
         loadParam(nh, "margin", scan_plan_params_.margin) &&
         loadParam(nh, "overlap", scan_plan_params_.overlap) &&
         loadParam(nh, "scan_width", scan_plan_params_.scan_width) &&
         loadParam(nh, "min_qa_value", scan_plan_params_.min_qa_value) &&
         loadParam(nh, "max_qa_value", scan_plan_params_.max_qa_value) &&
         loadParam(nh, "approach_distance", scan_plan_params_.approach_distance) &&
         loadParam(nh, "window_width", scan_plan_params_.window_width);
}

void SurfaceBlendingService::save_scan_parameters(const std::string& filename)
{
  if (!godel_param_helpers::toFile(filename, scan_plan_params_))
  {
    ROS_WARN_STREAM("Unable to save scan-plan parameters to: " << filename);
  }
}

void SurfaceBlendingService::publish_selected_surfaces_changed()
{
  godel_msgs::SelectedSurfacesChanged msg;
  msg.selected_surfaces.clear();
  surface_server_.get_selected_list(msg.selected_surfaces);
  selected_surf_changed_pub_.publish(msg);
}

bool SurfaceBlendingService::run_robot_scan(visualization_msgs::MarkerArray& surfaces)
{
  bool succeeded = true;

  // publishing scan path preview
  robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);

  // clear all results
  surface_detection_.clear_results();

  // saving parameters used

  ROS_INFO_STREAM("Starting scan");

  int scans_completed = robot_scan_.scan(false);
  if (scans_completed > 0)
  {
    ROS_INFO_STREAM("Scan points reached " << scans_completed);
    succeeded = find_surfaces(surfaces);
  }
  else
  {
    succeeded = false;
    ROS_ERROR_STREAM("Scan failed");
  }
  return succeeded;
}

bool SurfaceBlendingService::find_surfaces(visualization_msgs::MarkerArray& surfaces)
{
  bool succeeded = true;
  if (surface_detection_.find_surfaces())
  {
    // clear current surfaces
    surface_server_.remove_all_surfaces();

    // adding meshes to server
    std::vector<pcl::PolygonMesh> meshes;
    surface_detection_.get_meshes(meshes);
    for (std::size_t i = 0; i < meshes.size(); i++)
    {
      surface_server_.add_surface(meshes[i]);
    }

    // copying to surface markers to output argument
    visualization_msgs::MarkerArray markers_msg = surface_detection_.get_surface_markers();
    surfaces.markers.insert(surfaces.markers.begin(), markers_msg.markers.begin(),
                            markers_msg.markers.end());

    // saving latest successful results
    latest_surface_detection_results_.surface_detection = surface_detection_.params_;
    latest_surface_detection_results_.surfaces_found = true;
    latest_surface_detection_results_.surfaces = surfaces;
    robot_scan_.get_latest_scan_poses(latest_surface_detection_results_.robot_scan_poses);

    // saving region colored point cloud
    region_cloud_msg_ = sensor_msgs::PointCloud2();
    surface_detection_.get_region_colored_cloud(region_cloud_msg_);
  }
  else
  {
    succeeded = false;
    region_cloud_msg_ = sensor_msgs::PointCloud2();
  }

  return succeeded;
}

void SurfaceBlendingService::remove_previous_process_plan()
{
  // removing boundary markers
  visualization_msgs::MarkerArray& bds = process_path_results_.process_boundaries_;
  visualization_msgs::MarkerArray& paths = process_path_results_.process_paths_;
  visualization_msgs::MarkerArray& scans = process_path_results_.scan_paths_;

  for (std::size_t i = 0; i < bds.markers.size(); i++)
  {
    visualization_msgs::Marker& m = bds.markers[i];
    m.action = m.DELETE;
  }

  for (std::size_t i = 0; i < paths.markers.size(); i++)
  {
    visualization_msgs::Marker& m = paths.markers[i];
    m.action = m.DELETE;
  }

  for (std::size_t i = 0; i < scans.markers.size(); ++i)
  {
    visualization_msgs::Marker& m = scans.markers[i];
    m.action = m.DELETE;
  }

  // publishing markers for deletion
  visualization_msgs::MarkerArray markers;
  markers.markers.insert(markers.markers.end(), bds.markers.begin(), bds.markers.end());
  markers.markers.insert(markers.markers.end(), paths.markers.begin(), paths.markers.end());
  markers.markers.insert(markers.markers.end(), scans.markers.begin(), scans.markers.end());

  tool_path_markers_pub_.publish(markers);

  bds.markers.clear();
  paths.markers.clear();
  scans.markers.clear();
}

bool SurfaceBlendingService::animate_tool_path()
{
  bool succeeded = !process_path_results_.process_paths_.markers.empty();
  stop_tool_animation_ = true;

  ROS_INFO_STREAM("Tool animation activated");

  boost::thread(&SurfaceBlendingService::tool_animation_timer_callback, this);

  return succeeded;
}

static bool isBlendPath(const std::string& s)
{
  const static std::string prefix = "_blend";
  return s.find(prefix, s.size() - prefix.size()) != std::string::npos;
}

void SurfaceBlendingService::tool_animation_timer_callback()
{
  stop_tool_animation_ = false;

  // path progress color
  std_msgs::ColorRGBA green;
  green.a = 1.f;
  green.g = 1.f;
  green.r = green.b = 0.f;

  // marker array for all markers
  visualization_msgs::MarkerArray process_markers;

  // tool marker at arbitrary position
  visualization_msgs::MarkerArray tool_markers =
      create_tool_markers(geometry_msgs::Point(), geometry_msgs::Pose(), "world_frame");

  // Hacky thing to get it going for demo

  // adding markers
  int num_path_markers = process_path_results_.process_paths_.markers.size();
  // Blending Paths
  process_markers.markers.insert(process_markers.markers.end(),
                                 process_path_results_.process_paths_.markers.begin(),
                                 process_path_results_.process_paths_.markers.end());
  // Surface outlines
  process_markers.markers.insert(process_markers.markers.end(),
                                 process_path_results_.process_boundaries_.markers.begin(),
                                 process_path_results_.process_boundaries_.markers.end());
  // The 'tool'
  process_markers.markers.insert(process_markers.markers.end(), tool_markers.markers.begin(),
                                 tool_markers.markers.end());

  ROS_INFO_STREAM(process_path_results_.process_paths_.markers.size()
                  << " path markers, " << process_path_results_.process_boundaries_.markers.size()
                  << " boundary markers, " << tool_markers.markers.size() << " tool markers.");

  ros::Duration loop_pause(0.02f);
  for (int i = 0; i < num_path_markers; i++)
  {
    visualization_msgs::Marker& path_marker = process_markers.markers[i];

    for (std::size_t j = 0; j < path_marker.points.size(); j++)
    {
      if (stop_tool_animation_)
      {
        ROS_WARN_STREAM("tool path animation completed");
        return;
      }

      // updating path color at current point
      path_marker.colors[j] = green;

      // updating tool markers
      tool_markers =
          create_tool_markers(path_marker.points[j], path_marker.pose, path_marker.header.frame_id);
      int start_tool_marker_index = process_markers.markers.size() - tool_markers.markers.size();

      process_markers.markers.erase(
          boost::next(process_markers.markers.begin(), start_tool_marker_index),
          process_markers.markers.end());
      process_markers.markers.insert(process_markers.markers.end(), tool_markers.markers.begin(),
                                     tool_markers.markers.end());

      // publish marker array
      tool_path_markers_pub_.publish(process_markers);

      loop_pause.sleep();
    }
  }

  ROS_INFO_STREAM("tool path animation completed");
}

visualization_msgs::MarkerArray
SurfaceBlendingService::create_tool_markers(const geometry_msgs::Point& pos,
                                            const geometry_msgs::Pose& pose, std::string frame_id)
{
  visualization_msgs::MarkerArray tool;
  tool.markers.resize(2);
  visualization_msgs::Marker& disk = tool.markers.at(0);
  visualization_msgs::Marker& shaft = tool.markers.at(1);

  std_msgs::ColorRGBA blue;
  blue.r = 0.;
  blue.g = .1;
  blue.b = 1.;
  blue.a = 0.7;

  disk.action = visualization_msgs::Marker::ADD;
  disk.color = blue;
  disk.frame_locked = true;
  disk.header.frame_id = frame_id;
  disk.header.seq = 0;
  disk.header.stamp = ros::Time::now();
  disk.lifetime = ros::Duration(0.);
  disk.pose = pose;
  disk.ns = TOOL_NAMESPACE;
  // disk/shaft position filled out below
  disk.type = visualization_msgs::Marker::CYLINDER;
  shaft = disk;

  tf::Transform marker_pose;
  tf::poseMsgToTF(pose, marker_pose);

  disk.id = 0;
  tf::Vector3 marker_pos(pos.x, pos.y, pos.z + .5 * TOOL_THK);
  marker_pos = marker_pose * marker_pos;
  tf::pointTFToMsg(marker_pos, disk.pose.position);
  disk.scale.x = disk.scale.y = TOOL_DIA;
  disk.scale.z = TOOL_THK;

  shaft.id = 1;
  marker_pos = tf::Vector3(pos.x, pos.y, pos.z + TOOL_THK + 0.5 * TOOL_SHAFT_LEN);
  marker_pos = marker_pose * marker_pos;
  tf::pointTFToMsg(marker_pos, shaft.pose.position);
  shaft.scale.x = shaft.scale.y = TOOL_SHAFT_DIA;
  shaft.scale.z = TOOL_SHAFT_LEN;

  return tool;
}

bool SurfaceBlendingService::surface_detection_server_callback(
    godel_msgs::SurfaceDetection::Request& req, godel_msgs::SurfaceDetection::Response& res)
{

  res.surfaces_found = false;
  res.surfaces = visualization_msgs::MarkerArray();
  remove_previous_process_plan();

  switch (req.action)
  {

  case godel_msgs::SurfaceDetection::Request::PUBLISH_SCAN_PATH:

    if (req.use_default_parameters)
    {
      robot_scan_.params_ = default_robot_scan_params__;
    }
    else
    {
      robot_scan_.params_ = req.robot_scan;
    }

    robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);
    break;

  case godel_msgs::SurfaceDetection::Request::SCAN_AND_FIND_ONLY:

    if (req.use_default_parameters)
    {
      robot_scan_.params_ = default_robot_scan_params__;
      surface_detection_.params_ = default_surf_detection_params_;
    }
    else
    {
      robot_scan_.params_ = req.robot_scan;
      surface_detection_.params_ = req.surface_detection;
    }

    res.surfaces_found = run_robot_scan(res.surfaces);
    res.surfaces.markers.clear();
    break;

  case godel_msgs::SurfaceDetection::Request::SCAN_FIND_AND_RETURN:

    if (req.use_default_parameters)
    {
      robot_scan_.params_ = default_robot_scan_params__;
      surface_detection_.params_ = default_surf_detection_params_;
    }
    else
    {
      robot_scan_.params_ = req.robot_scan;
      surface_detection_.params_ = req.surface_detection;
    }

    res.surfaces_found = run_robot_scan(res.surfaces);
    break;

  case godel_msgs::SurfaceDetection::Request::FIND_ONLY:

    if (req.use_default_parameters)
    {
      surface_detection_.params_ = default_surf_detection_params_;
    }
    else
    {
      surface_detection_.params_ = req.surface_detection;
    }

    res.surfaces_found = find_surfaces(res.surfaces);
    res.surfaces.markers.clear();
    break;

  case godel_msgs::SurfaceDetection::Request::FIND_AND_RETURN:

    if (req.use_default_parameters)
    {
      surface_detection_.params_ = default_surf_detection_params_;
    }
    else
    {
      surface_detection_.params_ = req.surface_detection;
    }

    res.surfaces_found = find_surfaces(res.surfaces);
    break;

  case godel_msgs::SurfaceDetection::Request::RETURN_LATEST_RESULTS:

    res = latest_surface_detection_results_;
    break;
  }

  return true;
}

bool SurfaceBlendingService::select_surface_server_callback(godel_msgs::SelectSurface::Request& req,
                                                            godel_msgs::SelectSurface::Response&)
{
  switch (req.action)
  {
  case godel_msgs::SelectSurface::Request::SELECT:

    for (std::size_t i = 0; req.select_surfaces.size(); i++)
    {
      surface_server_.set_selection_flag(req.select_surfaces[i], true);
    }
    break;

  case godel_msgs::SelectSurface::Request::DESELECT:

    for (std::size_t i = 0; req.select_surfaces.size(); i++)
    {
      surface_server_.set_selection_flag(req.select_surfaces[i], false);
    }
    break;

  case godel_msgs::SelectSurface::Request::SELECT_ALL:

    surface_server_.select_all(true);
    break;

  case godel_msgs::SelectSurface::Request::DESELECT_ALL:

    surface_server_.select_all(false);
    break;

  case godel_msgs::SelectSurface::Request::HIDE_ALL:

    surface_server_.show_all(false);
    break;

  case godel_msgs::SelectSurface::Request::SHOW_ALL:
    surface_server_.show_all(true);
    break;
  }

  return true;
}

bool SurfaceBlendingService::process_path_server_callback(
    godel_msgs::ProcessPlanning::Request& req, godel_msgs::ProcessPlanning::Response& res)
{
  godel_process_path_generation::VisualizeBlendingPlan process_plan;
  process_plan.request.params =
      req.use_default_parameters ? default_blending_plan_params_ : req.params;
  godel_msgs::ScanPlanParameters scan_params =
      req.use_default_parameters ? default_scan_params_ : req.scan_params;
  switch (req.action)
  {
  case godel_msgs::ProcessPlanning::Request::GENERATE_MOTION_PLAN:
  case godel_msgs::ProcessPlanning::Request::GENERATE_MOTION_PLAN_AND_PREVIEW:
    trajectory_library_ = generateMotionLibrary(process_plan.request.params, scan_params);
    visualizePaths();
    break;

  case godel_msgs::ProcessPlanning::Request::PREVIEW_TOOL_PATH:

    res.succeeded = animate_tool_path();
    break;

  case godel_msgs::ProcessPlanning::Request::PREVIEW_MOTION_PLAN:

    res.succeeded = false;
    break;

  case godel_msgs::ProcessPlanning::Request::EXECUTE_MOTION_PLAN:

    res.succeeded = false;
    break;

  default:

    ROS_ERROR_STREAM("Unknown action code '" << req.action << "' request");
    break;
  }

  res.succeeded = false;
  return true;
}

bool SurfaceBlendingService::surface_blend_parameters_server_callback(
    godel_msgs::SurfaceBlendingParameters::Request& req,
    godel_msgs::SurfaceBlendingParameters::Response& res)
{
  switch (req.action)
  {
  case godel_msgs::SurfaceBlendingParameters::Request::GET_CURRENT_PARAMETERS:

    res.surface_detection = surface_detection_.params_;
    res.robot_scan = robot_scan_.params_;
    res.blending_plan = blending_plan_params_;
    res.scan_plan = scan_plan_params_;
    break;

  case godel_msgs::SurfaceBlendingParameters::Request::GET_DEFAULT_PARAMETERS:

    res.surface_detection = default_surf_detection_params_;
    res.robot_scan = default_robot_scan_params__;
    res.blending_plan = default_blending_plan_params_;
    res.scan_plan = default_scan_params_;
    break;

  // Update the current parameters in this service
  case godel_msgs::SurfaceBlendingParameters::Request::SET_PARAMETERS:
  case godel_msgs::SurfaceBlendingParameters::Request::SAVE_PARAMETERS:
    surface_detection_.params_ = req.surface_detection;
    robot_scan_.params_ = req.robot_scan;
    blending_plan_params_ = req.blending_plan;
    scan_plan_params_ = req.scan_plan;

    if (req.action == godel_msgs::SurfaceBlendingParameters::Request::SAVE_PARAMETERS)
    {
      this->save_blend_parameters(param_cache_prefix_ + BLEND_PARAMS_FILE);
      this->save_scan_parameters(param_cache_prefix_ + SCAN_PARAMS_FILE);
      robot_scan_.save_parameters(param_cache_prefix_ + ROBOT_SCAN_PARAMS_FILE);
      surface_detection_.save_parameters(param_cache_prefix_ + SURFACE_DETECTION_PARAMS_FILE);
    }
    break;
  }

  return true;
}

bool SurfaceBlendingService::selectMotionPlanCallback(godel_msgs::SelectMotionPlan::Request& req,
                                                      godel_msgs::SelectMotionPlan::Response& res)
{
  // Check to ensure the plan exists
  if (trajectory_library_.get().find(req.name) == trajectory_library_.get().end())
  {
    ROS_WARN_STREAM("Motion plan " << req.name << " does not exist. Cannot execute.");
    res.code = godel_msgs::SelectMotionPlan::Response::NO_SUCH_NAME;
    return false;
  }

  bool is_blend = trajectory_library_.get()[req.name].type == godel_msgs::ProcessPlan::BLEND_TYPE;

  if (is_blend)
  {
    godel_msgs::BlendProcessExecution srv;
    srv.request.trajectory_approach = trajectory_library_.get()[req.name].trajectory_approach;
    srv.request.trajectory_process = trajectory_library_.get()[req.name].trajectory_process;
    srv.request.trajectory_depart = trajectory_library_.get()[req.name].trajectory_depart;
    srv.request.wait_for_execution = req.wait_for_execution;
    srv.request.simulate = req.simulate;

    if (!blend_process_client_.call(srv))
    {
      ROS_ERROR_STREAM("Could not call blend process client");
      return false;
    }
  }
  else
  {
    godel_msgs::KeyenceProcessExecution srv;
    srv.request.trajectory_approach = trajectory_library_.get()[req.name].trajectory_approach;
    srv.request.trajectory_process = trajectory_library_.get()[req.name].trajectory_process;
    srv.request.trajectory_depart = trajectory_library_.get()[req.name].trajectory_depart;
    srv.request.wait_for_execution = req.wait_for_execution;
    srv.request.simulate = req.simulate;

    if (!scan_process_client_.call(srv))
    {
      ROS_ERROR_STREAM("Could not call scan process client");
      return false;
    }
  }

  return true;
}

bool SurfaceBlendingService::getMotionPlansCallback(
    godel_msgs::GetAvailableMotionPlans::Request&,
    godel_msgs::GetAvailableMotionPlans::Response& res)
{
  typedef godel_surface_detection::TrajectoryLibrary::TrajectoryMap::const_iterator MapIter;
  for (MapIter it = trajectory_library_.get().begin(); it != trajectory_library_.get().end(); ++it)
  {
    res.names.push_back(it->first);
  }
  return true;
}

bool SurfaceBlendingService::loadSaveMotionPlanCallback(
    godel_msgs::LoadSaveMotionPlan::Request& req, godel_msgs::LoadSaveMotionPlan::Response& res)
{
  switch (req.mode)
  {
  case godel_msgs::LoadSaveMotionPlan::Request::MODE_LOAD:
    trajectory_library_.load(req.path);
    break;

  case godel_msgs::LoadSaveMotionPlan::Request::MODE_SAVE:
    trajectory_library_.save(req.path);
    break;
  }

  res.code = godel_msgs::LoadSaveMotionPlan::Response::SUCCESS;
  return true;
}

bool SurfaceBlendingService::renameSurfaceCallback(godel_msgs::RenameSurface::Request& req,
                                                   godel_msgs::RenameSurface::Response& res)
{
  if (surface_server_.rename_surface(req.old_name, req.new_name))
  {
    ROS_INFO_STREAM("Surface re-name successful");
    return true;
  }
  else
  {
    ROS_WARN_STREAM("Surface rename failed");
    return false;
  }
}

void SurfaceBlendingService::visualizePaths()
{
  visualization_msgs::MarkerArray paths;
  paths.markers.insert(paths.markers.end(), process_path_results_.process_paths_.markers.begin(),
                       process_path_results_.process_paths_.markers.end());
  paths.markers.insert(paths.markers.end(), process_path_results_.scan_paths_.markers.begin(),
                       process_path_results_.scan_paths_.markers.end());
  tool_path_markers_pub_.publish(paths);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "surface_blending_service");
  ros::AsyncSpinner spinner(4);
  spinner.start();
  SurfaceBlendingService service;
  ROS_INFO("INIT BLENDING SERVICE");
  if (service.init())
  {
    ROS_INFO("BLENDING SERVICE INIT SUCCESSFUL!");
    service.run();
  }

  ros::waitForShutdown();
}
