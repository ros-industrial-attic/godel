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

ProcessPlanResult generateProcessPlan(const std::string& name, 
                                      const visualization_msgs::Marker& path)
{
  return {};
}

bool SurfaceBlendingService::generate_process_plan(godel_process_path_generation::VisualizeBlendingPlan &process_plan)
{
  // creating color structures for the different parts for the process path
  std_msgs::ColorRGBA yellow;
  yellow.a = 1.f;
  yellow.b = 0.f;
  yellow.r = yellow.g = 1.f;

  // saving parameters
  blending_plan_params_ = process_plan.request.params;

  // clear previous results
  process_path_results_.process_boundaries_.markers.clear();
  process_path_results_.process_paths_.markers.clear();
  process_path_results_.tool_parts_.markers.clear();
  process_path_results_.scan_paths_.markers.clear();

  // generating boundaries
  std::vector<pcl::PolygonMesh> meshes;
  surface_server_.get_selected_surfaces(meshes);
  std::vector<geometry_msgs::Polygon> &boundaries = process_plan.request.surface.boundaries;

  int boundaries_found = 0; // successful computations counter
  int marker_counter = 0;

  // duration parameters for generated plans
  duration_results_.clear();

  for(std::size_t i =0;i <meshes.size();i++)
  {
    // markers for saving individual mesh results
    visualization_msgs::MarkerArray boundary_markers;
    geometry_msgs::Pose boundary_pose;
    visualization_msgs::Marker path_marker;
    const pcl::PolygonMesh &mesh = meshes[i];
    //if(mesh_importer_.calculateBoundaryData(mesh))
    if(mesh_importer_.calculateSimpleBoundary(mesh))
    {
      // Filter out boundaries that are too small (assumed to be machine-vision artifacts) and improperly formed
      godel_process_path::PolygonBoundaryCollection filtered_boundaries;
      BOOST_FOREACH(const godel_process_path::PolygonBoundary &bnd, mesh_importer_.getBoundaries())
      {
        double circ = godel_process_path::polygon_utils::circumference(bnd);
        if (circ < blending_plan_params_.min_boundary_length)
        {
          ROS_WARN_STREAM("Ignoring boundary with length " << circ);
        }
        else if (!godel_process_path::polygon_utils::checkBoundary(bnd))
        {
          ROS_WARN_STREAM("Ignoring ill-formed boundary");
        }
        else
        {
          ROS_INFO_STREAM("Boundary has circumference " << circ);
          filtered_boundaries.push_back(bnd);
        }
      }
      BOOST_FOREACH(godel_process_path::PolygonBoundary &bnd, filtered_boundaries)
      {
        // Filter and reverse boundaries
        godel_process_path::polygon_utils::filter(bnd, 0.1);
        std::reverse(bnd.begin(), bnd.end());
      }

      // DEBUG
      godel_process_path::PolygonBoundaryCollection coll;
      BOOST_FOREACH(godel_process_path::PolygonBoundary &bnd, filtered_boundaries)
      {
        godel_surface_detection::ProfilimeterScanParams param;
        param.width_ = 0.02;
        param.overlap_ = 0.0;

        // Filter and reverse boundaries
        coll.push_back(godel_surface_detection::generateProfilimeterScanPath(bnd, param));
      }
      visualization_msgs::MarkerArray scan_markers;
      std_msgs::ColorRGBA blue;
      blue.b = 1.0;
      blue.r = 0.0;
      blue.g = 0.0;
      godel_process_path::utils::translations::godelToVisualizationMsgs(scan_markers, coll, yellow,.0005);


      // create boundaries markers
      godel_process_path::utils::translations::godelToVisualizationMsgs(boundary_markers,filtered_boundaries
          ,yellow,.0005);
      mesh_importer_.getPose(boundary_pose);

      // setting boundary marker properties
      for(std::size_t j =0; j < boundary_markers.markers.size();j++)
      {
        visualization_msgs::Marker &m = boundary_markers.markers[j];
        m.header.frame_id = mesh.header.frame_id;
        m.id = marker_counter;
        m.lifetime = ros::Duration(0);
        m.ns = BOUNDARY_NAMESPACE;
        m.pose = boundary_pose;
        m.points.push_back(m.points.front());   // Close polygon loop for visualization
        m.colors.push_back(m.colors.front());

        marker_counter++;
      }
      tool_path_markers_pub_.publish(boundary_markers);       // Pre-publish boundaries before completing offset

      // Publish the raster paths
      for(std::size_t j =0; j < scan_markers.markers.size();j++)
      {
        visualization_msgs::Marker &m = scan_markers.markers[j];
        if (m.points.empty()) continue;
        m.header.frame_id = mesh.header.frame_id;
        m.id = marker_counter;
        m.lifetime = ros::Duration(0);
        m.ns = BOUNDARY_NAMESPACE;
        m.pose = boundary_pose;
        marker_counter++;
      }
      tool_path_markers_pub_.publish(scan_markers);

      // add boundaries to request
      boundaries.clear();
      godel_process_path::utils::translations::godelToGeometryMsgs(boundaries, filtered_boundaries);
      tf::poseTFToMsg(tf::Transform::getIdentity(),process_plan.request.surface.pose);

      // calling visualization service and saving results
      if(visualize_process_path_client_.call(process_plan))
      {
        // create path marker
        path_marker = process_plan.response.path;
        path_marker.header.frame_id = mesh.header.frame_id;
        path_marker.id = marker_counter;
        path_marker.lifetime = ros::Duration(0);
        path_marker.ns = PATH_NAMESPACE;
        path_marker.pose = boundary_pose;

        // saving into results structure;
        process_path_results_.process_boundaries_.markers.insert(process_path_results_.process_boundaries_.markers.end(),
            boundary_markers.markers.begin(),boundary_markers.markers.end());
        process_path_results_.process_paths_.markers.push_back(path_marker);
        // save laser scan data
        process_path_results_.scan_paths_.markers.insert(process_path_results_.scan_paths_.markers.end(),
          scan_markers.markers.begin(), scan_markers.markers.end());

        duration_results_.push_back(process_plan.response.sleep_times);

        boundaries_found++;
        marker_counter++;
      }
    }
  }


  return boundaries_found >0;
}

void SurfaceBlendingService::scan_planning_timer_callback(const ros::TimerEvent&)
{
  ROS_WARN_STREAM("Scan planning callback!");
  std::vector<trajectory_msgs::JointTrajectory> trajectories;
  for (size_t i = 0; i < process_path_results_.scan_paths_.markers.size(); ++i)
  {
    godel_msgs::TrajectoryPlanning plan;
    // Set planning parameters    
    plan.request.group_name = SCAN_TRAJECTORY_GROUP_NAME;
    plan.request.tool_frame = SCAN_TRAJECTORY_TOOL_FRAME;
    plan.request.world_frame = SCAN_TRAJECTORY_WORLD_FRAME;
    plan.request.angle_discretization = SCAN_TRAJECTORY_ANGLE_DISC;
    plan.request.interpoint_delay = SCAN_TRAJECTORY_INTERPOINT_DELAY;
    plan.request.is_blending_path = false;
    plan.request.plan_from_current_position = false;
    
    const visualization_msgs::Marker& marker = process_path_results_.scan_paths_.markers[i];

    plan.request.path.reference = marker.pose;
    plan.request.path.points = marker.points;
    plan.request.path.durations = duration_results_[i];

    ROS_INFO_STREAM("Calling trajectory planner for scan process " << i);
    if (trajectory_planner_client_.call(plan))
    {
      ROS_INFO_STREAM("Trajectory planner succeeded for scan plan " << i);
      trajectories.push_back(plan.response.trajectory);
    }
    else
    {
      ROS_WARN_STREAM("Failed to find trajectory plan for scan plan " << i);
    }
  }

  ROS_INFO_STREAM("Trajectory scan planning complete");

  // save to file for easier testing
  if (trajectories.size() > 0)
  {
    ROS_INFO_STREAM("Saving trajectories to bagfile: " << SCAN_TRAJECTORY_BAGFILE);
    rosbag::Bag bag;
    bag.open(SCAN_TRAJECTORY_BAGFILE, rosbag::bagmode::Write);
    for (std::size_t i = 0; i < trajectories.size(); ++i)
    {
      bag.write("trajectory", ros::Time::now(), trajectories[i]);
    }
  }
}

void SurfaceBlendingService::trajectory_planning_timer_callback(const ros::TimerEvent&)
{
  // Container representing trajectories for each selected surface
  std::vector<trajectory_msgs::JointTrajectory> trajectories;

  ROS_INFO_STREAM("Creating trajectory plan");

  for (size_t i = 0; i < process_path_results_.process_paths_.markers.size(); ++i)
  {
    godel_msgs::TrajectoryPlanning plan;
    // Set planning parameters    
    plan.request.group_name = BLEND_TRAJECTORY_GROUP_NAME;
    plan.request.tool_frame = BLEND_TRAJECTORY_TOOL_FRAME;
    plan.request.world_frame = BLEND_TRAJECTORY_WORLD_FRAME;
    plan.request.angle_discretization = BLEND_TRAJECTORY_ANGLE_DISC;
    plan.request.interpoint_delay = BLEND_TRAJECTORY_INTERPOINT_DELAY;
    plan.request.is_blending_path = true;
    plan.request.plan_from_current_position = true;

    const visualization_msgs::Marker& marker = process_path_results_.process_paths_.markers[i];

    plan.request.path.reference = marker.pose;
    plan.request.path.points = marker.points;
    plan.request.path.durations = duration_results_[i];

    ROS_INFO_STREAM("Calling trajectory planner for surface process " << i);
    if (trajectory_planner_client_.call(plan))
    {
      ROS_INFO_STREAM("Trajectory planner succeeded for plan " << i);
      trajectories.push_back(plan.response.trajectory);
    }
    else
    {
      ROS_WARN_STREAM("Failed to find trajectory plan for surface plan " << i);
    }
  }

  ROS_INFO_STREAM("Trajectory planning complete");

  // save to file for easier testing
  if (trajectories.size() > 0)
  {
    ROS_INFO_STREAM("Saving trajectories to bagfile: " << BLEND_TRAJECTORY_BAGFILE);
    rosbag::Bag bag;
    bag.open(BLEND_TRAJECTORY_BAGFILE, rosbag::bagmode::Write);
    for (std::size_t i = 0; i < trajectories.size(); ++i)
    {
      bag.write("trajectory", ros::Time::now(), trajectories[i]);
    }
  }

  // After the blend path completes, signal the profilimeter planning function to run
  scan_planning_timer_.setPeriod(ros::Duration(0.1f));
  scan_planning_timer_.start();
}