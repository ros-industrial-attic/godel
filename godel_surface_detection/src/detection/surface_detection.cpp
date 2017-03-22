/*
        Copyright Feb 11, 2014 Southwest Research Institute

        Licensed under the Apache License, Version 2.0 (the "License");
        you may not use this file except in compliance with the License.
        You may obtain a copy of the License at

                http://www.apache.org/licenses/LICENSE-2.0

        Unless required by applicable law or agreed to in writing, software
        distributed under the License is distributed on an "AS IS" BASIS,
        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        See the License for the specific language governing permissions and
        limitations under the License.
*/

#include <detection/surface_detection.h>
#include <godel_param_helpers/godel_param_helpers.h>
#include <meshing_plugins_base/meshing_base.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_loader.h>
#include <segmentation/surface_segmentation.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf/transform_datatypes.h>
#include <utils/mesh_conversions.h>
#include <swri_profiler/profiler.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/pcl_base.h>
#include <pcl/filters/passthrough.h>

namespace godel_surface_detection
{
namespace detection
{

namespace defaults
{

// static const double ACQUISITION_TIME = 5.0f;
static std::string FRAME_ID = "world_frame";

static const int STATISTICAL_OUTLIER_MEAN = 50;
static const double STATISTICAL_OUTLIER_STDEV_THRESHOLD = 1;
static const int K_SEARCH = 50;

static const int REGION_GROWING_MIN_CLUSTER_SIZE = 100;
static const int REGION_GROWING_MAX_CLUSTER_SIZE = 100000;
static const int REGION_GROWING_NEIGHBORS = 50;
static const double REGION_GROWING_SMOOTHNESS_THRESHOLD = (M_PI / 180.0f) * 7.0f;
static const double REGION_GROWING_CURVATURE_THRESHOLD = 1.0f;

static const double TRIANGULATION_SEARCH_RADIUS = 0.01f;
static const double TRIANGULATION_MU = 2.5f;
static const int TRIANGULATION_MAX_NEAREST_NEIGHBORS = 100;
static const double TRIANGULATION_MAX_SURFACE_ANGLE = M_PI / 4.0f;
static const double TRIANGULATION_MIN_ANGLE = M_PI / 18.0f;
static const double TRIANGULATION_MAX_ANGLE = 2.0f * M_PI / 3.0f;
static const bool TRIANGULATION_NORMAL_CONSISTENCY = false;

static const bool PLANE_APROX_REFINEMENT_ENABLED = true;
static const int PLANE_APROX_REFINEMENT_SEG_MAX_ITERATIONS = 100;
static const double PLANE_APROX_REFINEMENT_SEG_DIST_THRESHOLD = 0.01f;
static const double PLANE_APROX_REFINEMENT_SAC_PLANE_DISTANCE = 0.01f;
static const std::string PLANE_APROX_REFINEMENT_KDTREE_RADIUS = "pa_kdtree_radius";

static const double VOXEL_LEAF_SIZE = 0.01f;

static const double OCCUPANCY_THRESHOLD = 0.1f;

// Moving least square smoothing
static const double MLS_UPSAMPLING_RADIUS = 0.01f;
static const double MLS_SEARCH_RADIUS = 0.01f;
static const int MLS_POINT_DENSITY = 40;

static const bool USE_TABLETOP_SEGMENTATION = true;
static const double TABLETOP_SEG_DISTANCE_THRESH = 0.005f;

static const double MARKER_ALPHA = 1.0f;
}

namespace config
{
static const std::string POINT_COLUD_TOPIC = "sensor_point_cloud";
}

namespace params
{
static const std::string FRAME_ID = "frame_id";
static const std::string USE_OCTOMAP = "use_octomap";

static const std::string STOUTLIER_MEAN = "stout_mean";
static const std::string STOUTLIER_STDEV_THRESHOLD = "stout_stdev_threshold";
static const std::string K_SEARCH = "k_search";

static const std::string REGION_GROWING_MIN_CLUSTER_SIZE = "rg_min_cluster_size";
static const std::string REGION_GROWING_MAX_CLUSTER_SIZE = "rg_max_cluster_size";
static const std::string REGION_GROWING_NEIGHBORS = "rg_neighbors";
static const std::string REGION_GROWING_SMOOTHNESS_THRESHOLD = "rg_smoothness_threshold";
static const std::string REGION_GROWING_CURVATURE_THRESHOLD = "rg_curvature_threshold";

static const std::string TRIANGULATION_SEARCH_RADIUS = "tr_search_radius";
static const std::string TRIANGULATION_MU = "tr_mu";
static const std::string TRIANGULATION_MAX_NEAREST_NEIGHBORS = "tr_nearest_neighbors";
static const std::string TRIANGULATION_MAX_SURFACE_ANGLE = "tr_max_surface_angle";
static const std::string TRIANGULATION_MIN_ANGLE = "tr_min_angle";
static const std::string TRIANGULATION_MAX_ANGLE = "tr_max_angle";
static const std::string TRIANGULATION_NORMAL_CONSISTENCY = "tr_normal_consistency";

static const std::string PLANE_APROX_REFINEMENT_ENABLED = "pa_enabled";
static const std::string PLANE_APROX_REFINEMENT_SEG_MAX_ITERATIONS = "pa_seg_max_iterations";
static const std::string PLANE_APROX_REFINEMENT_SEG_DIST_THRESHOLD = "pa_seg_dist_threshold";
static const std::string PLANE_APROX_REFINEMENT_SAC_PLANE_DISTANCE = "pa_sac_plane_distance";
static const std::string PLANE_APROX_REFINEMENT_KDTREE_RADIUS = "pa_kdtree_radius";

static const std::string VOXEL_LEAF_SIZE = "voxel_leaf";

static const std::string OCCUPANCY_THRESHOLD = "occupancy_threshold";

static const std::string MLS_UPSAMPLING_RADIUS = "mls_upsampling_radius";
static const std::string MLS_SEARCH_RADIUS = "mls_search_radius";
static const std::string MLS_POINT_DENSITY = "mls_point_density";

static const std::string USE_TABLETOP_SEGMENTATION = "use_tabletop_segmentation";
static const std::string TABLETOP_SEG_DISTANCE_THRESH = "tabletop_seg_distance_thresh";

static const std::string MARKER_ALPHA = "marker_alpha";
}
}
}

static const float INPUT_CLOUD_VOXEL_FILTER_SIZE = 0.0015;
const static int DOWNSAMPLE_NUMBER = 3;
const static std::string MESHING_PLUGIN_PARAM = "meshing_plugin_name";

namespace godel_surface_detection
{
  namespace detection
  {
    SurfaceDetection::SurfaceDetection()
      : full_cloud_ptr_(new CloudRGB())
      , process_cloud_ptr_(new CloudRGB())
      , acquired_clouds_counter_(0)
      , random_engine_(0) // This is using a fixed seed for down-sampling at the moment
    {
      params_.frame_id = defaults::FRAME_ID;
      params_.k_search = defaults::K_SEARCH;
      params_.meanK = defaults::STATISTICAL_OUTLIER_MEAN;
      params_.stdv_threshold = defaults::STATISTICAL_OUTLIER_STDEV_THRESHOLD;
      params_.rg_min_cluster_size = defaults::REGION_GROWING_MIN_CLUSTER_SIZE;
      params_.rg_max_cluster_size = defaults::REGION_GROWING_MAX_CLUSTER_SIZE;
      params_.rg_neightbors = defaults::REGION_GROWING_NEIGHBORS;
      params_.rg_smoothness_threshold = defaults::REGION_GROWING_SMOOTHNESS_THRESHOLD;
      params_.rg_curvature_threshold = defaults::REGION_GROWING_CURVATURE_THRESHOLD;
      params_.tr_search_radius = defaults::TRIANGULATION_SEARCH_RADIUS;
      params_.tr_mu = defaults::TRIANGULATION_MU;
      params_.tr_max_nearest_neighbors = defaults::TRIANGULATION_MAX_NEAREST_NEIGHBORS;
      params_.tr_max_surface_angle = defaults::TRIANGULATION_MAX_SURFACE_ANGLE;
      params_.tr_min_angle = defaults::TRIANGULATION_MIN_ANGLE;
      params_.tr_max_angle = defaults::TRIANGULATION_MAX_ANGLE;
      params_.tr_normal_consistency = defaults::TRIANGULATION_NORMAL_CONSISTENCY;
      params_.voxel_leafsize = defaults::VOXEL_LEAF_SIZE;
      params_.marker_alpha = defaults::MARKER_ALPHA;
      params_.occupancy_threshold = defaults::OCCUPANCY_THRESHOLD;
      params_.mls_upsampling_radius = defaults::MLS_UPSAMPLING_RADIUS;
      params_.mls_point_density = defaults::MLS_POINT_DENSITY;
      params_.mls_search_radius = defaults::MLS_SEARCH_RADIUS;
      params_.use_tabletop_seg = defaults::USE_TABLETOP_SEGMENTATION;
      params_.tabletop_seg_distance_threshold = defaults::TABLETOP_SEG_DISTANCE_THRESH;
    }

    bool SurfaceDetection::init()
    {
      full_cloud_ptr_->header.frame_id = params_.frame_id;
      process_cloud_ptr_->header.frame_id = params_.frame_id;
      acquired_clouds_counter_ = 0;
      return true;
    }

    void SurfaceDetection::clear_results()
    {
      acquired_clouds_counter_ = 0;
      full_cloud_ptr_->clear();
      process_cloud_ptr_->clear();
      surface_clouds_.clear();
      mesh_markers_.markers.clear();
      meshes_.clear();
    }

    bool SurfaceDetection::load_parameters(const std::string& filename)
    {
      using godel_param_helpers::loadParam;
      using godel_param_helpers::loadBoolParam;

      if (godel_param_helpers::fromFile(filename, params_))
      {
        return true;
      }
      ros::NodeHandle nh("~/surface_detection");
      return loadParam(nh, params::FRAME_ID, params_.frame_id) &&
             loadParam(nh, params::K_SEARCH, params_.k_search) &&

             loadParam(nh, params::STOUTLIER_MEAN, params_.meanK) &&
             loadParam(nh, params::STOUTLIER_STDEV_THRESHOLD, params_.stdv_threshold) &&

             loadParam(nh, params::REGION_GROWING_MIN_CLUSTER_SIZE, params_.rg_min_cluster_size) &&
             loadParam(nh, params::REGION_GROWING_MAX_CLUSTER_SIZE, params_.rg_max_cluster_size) &&
             loadParam(nh, params::REGION_GROWING_NEIGHBORS, params_.rg_neightbors) &&
             loadParam(nh, params::REGION_GROWING_SMOOTHNESS_THRESHOLD,
                       params_.rg_smoothness_threshold) &&
             loadParam(nh, params::REGION_GROWING_CURVATURE_THRESHOLD,
                       params_.rg_curvature_threshold) &&

             loadParam(nh, params::PLANE_APROX_REFINEMENT_SEG_MAX_ITERATIONS,
                       params_.pa_seg_max_iterations) &&
             loadParam(nh, params::PLANE_APROX_REFINEMENT_SEG_DIST_THRESHOLD,
                       params_.pa_seg_dist_threshold) &&
             loadParam(nh, params::PLANE_APROX_REFINEMENT_SAC_PLANE_DISTANCE,
                       params_.pa_sac_plane_distance) &&
             loadParam(nh, params::PLANE_APROX_REFINEMENT_KDTREE_RADIUS, params_.pa_kdtree_radius) &&
             loadBoolParam(nh, params::PLANE_APROX_REFINEMENT_ENABLED, params_.pa_enabled) &&

             loadParam(nh, params::VOXEL_LEAF_SIZE, params_.voxel_leafsize) &&
             loadParam(nh, params::OCCUPANCY_THRESHOLD, params_.occupancy_threshold) &&

             loadParam(nh, params::MLS_UPSAMPLING_RADIUS, params_.mls_upsampling_radius) &&
             loadParam(nh, params::MLS_POINT_DENSITY, params_.mls_point_density) &&
             loadParam(nh, params::MLS_SEARCH_RADIUS, params_.mls_search_radius) &&

             loadBoolParam(nh, params::USE_TABLETOP_SEGMENTATION, params_.use_tabletop_seg) &&
             loadParam(nh, params::TABLETOP_SEG_DISTANCE_THRESH,
                       params_.tabletop_seg_distance_threshold) &&
             loadParam(nh, params::MARKER_ALPHA, params_.marker_alpha);
    }

    void SurfaceDetection::save_parameters(const std::string& filename)
    {
      if (!godel_param_helpers::toFile(filename, params_))
      {
        ROS_WARN_STREAM("Unable to save surface-detection parameters to: " << filename);
      }
    }

    void SurfaceDetection::mesh_to_marker(const pcl::PolygonMesh& mesh,
                                          visualization_msgs::Marker& marker,
                                          std::default_random_engine& random_engine)
    {
      // color value ranges
      std_msgs::ColorRGBA color;
      color.a = 1;

      // set marker properties
      tf::poseTFToMsg(tf::Transform::getIdentity(), marker.pose);
      marker.scale.x = marker.scale.y = marker.scale.z = 1;
      marker.type = marker.TRIANGLE_LIST;
      marker.action = marker.ADD;

      // create random color
      std::uniform_real_distribution<float> color_dist(0.5f, 1.0f);
      color.r = 0.5f + color_dist(random_engine);
      color.g = 0.5f + color_dist(random_engine);
      color.b = 0.5f + color_dist(random_engine);

      marker.color = color;

      // filling points
      meshToTrianglePoints(mesh, marker.points);
    }

    void SurfaceDetection::add_cloud(CloudRGB& cloud)
    {
      (*full_cloud_ptr_) += cloud;
      acquired_clouds_counter_++;
    }

    int SurfaceDetection::get_acquired_clouds_count() { return acquired_clouds_counter_; }


    visualization_msgs::MarkerArray SurfaceDetection::get_surface_markers()
    {
      return visualization_msgs::MarkerArray(mesh_markers_);
    }


    void SurfaceDetection::get_meshes(std::vector<pcl::PolygonMesh>& meshes)
    {
      meshes.insert(meshes.end(), meshes_.begin(), meshes_.end());
    }


    void SurfaceDetection::get_surface_clouds(std::vector<CloudRGB::Ptr>& surfaces)
    {
      surfaces.insert(surfaces.end(), surface_clouds_.begin(), surface_clouds_.end());
    }

    void SurfaceDetection::get_full_cloud(CloudRGB& cloud)
    {
      pcl::copyPointCloud(*full_cloud_ptr_, cloud);
    }

    void SurfaceDetection::get_full_cloud(sensor_msgs::PointCloud2 cloud_msg)
    {
      pcl::toROSMsg(*full_cloud_ptr_, cloud_msg);
    }

    void SurfaceDetection::get_process_cloud(CloudRGB& cloud)
    {
      pcl::copyPointCloud(*process_cloud_ptr_, cloud);
    }

    void SurfaceDetection::get_process_cloud(sensor_msgs::PointCloud2 &cloud_msg)
    {
      pcl::toROSMsg(*process_cloud_ptr_, cloud_msg);
    }

    void SurfaceDetection::get_region_colored_cloud(CloudRGB& cloud)
    {
      pcl::copyPointCloud(*region_colored_cloud_ptr_, cloud);
      cloud.header.frame_id = params_.frame_id;
    }


    void SurfaceDetection::get_region_colored_cloud(sensor_msgs::PointCloud2& cloud_msg)
    {
      pcl::toROSMsg(*region_colored_cloud_ptr_, cloud_msg);
      cloud_msg.header.frame_id = params_.frame_id;
    }

    bool SurfaceDetection::find_surfaces()
    {
      SWRI_PROFILE("find-surfaces");

      // Reset members
      surface_clouds_.clear();
      mesh_markers_.markers.clear();
      meshes_.clear();

      // Ensure cloud ptr is not empty
      if (full_cloud_ptr_->empty())
        return false;

      filterFullCloud();

      // Segment the part into surface clusters using a "region growing" scheme
      SurfaceSegmentation SS(process_cloud_ptr_);
      region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
      {
        SWRI_PROFILE("segment-clouds");
        SS.computeSegments(region_colored_cloud_ptr_);
      }
      SS.getSurfaceClouds(surface_clouds_);

      // Load the code to perform meshing dynamically
      pluginlib::ClassLoader<meshing_plugins_base::MeshingBase>
          poly_loader("meshing_plugins_base", "meshing_plugins_base::MeshingBase");
      boost::shared_ptr<meshing_plugins_base::MeshingBase> mesher;

      try
      {
        mesher = poly_loader.createInstance(getMeshingPluginName());
      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        return false;
      }

      // Compute mesh from point clouds
      SWRI_PROFILE("mesh-clouds");
      for (std::size_t i = 0; i < surface_clouds_.size(); i++)
      {
        pcl::PolygonMesh mesh;
        visualization_msgs::Marker marker;
        mesher->init(*surface_clouds_[i]);

        if (mesher->generateMesh(mesh))
        {
          // Create marker from mesh
          mesh_to_marker(mesh, marker, random_engine_);

          // saving other properties
          marker.header.frame_id = mesh.header.frame_id = surface_clouds_[i]->header.frame_id;
          marker.id = i;
          marker.color.a = params_.marker_alpha;

          // Push marker to mesh_markers_
          ROS_INFO_STREAM("Adding a marker for mesh with " + std::to_string(marker.points.size()) + " points");
          mesh_markers_.markers.push_back(marker);

          // Push mesh to meshes_
          meshes_.push_back(mesh);
        }
        else
        {
          ROS_WARN_STREAM("Meshing of segmented surface " << i << " failed.");
        }
      }

      return true;
    }

    std::string SurfaceDetection::getMeshingPluginName() const
    {
      ros::NodeHandle pnh ("~");
      std::string name;
      if (!pnh.getParam(MESHING_PLUGIN_PARAM, name))
      {
        ROS_WARN("Unable to load meshing plugin name from ros param '%s'",
                 MESHING_PLUGIN_PARAM.c_str());
      }

      return name;
    }

    void SurfaceDetection::filterFullCloud()
    {
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr intermediate_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

      //remove the table using the passthrough filter
      pcl::PassThrough<pcl::PointXYZRGB> pass;
      pass.setInputCloud(full_cloud_ptr_);
      const std::string FILTER_DIRECTION = "z";
      pass.setFilterFieldName (FILTER_DIRECTION);
      //keep poin clouds with these limits
      const double MINIMUM_DISTANCE = 0.01; // 1 cm
      const double MAXIMUM_DISTANCE = 1.0; // 1 m
      pass.setFilterLimits (MINIMUM_DISTANCE, MAXIMUM_DISTANCE);
      pass.filter (*intermediate_cloud_ptr);

      //downsample the full cloud using the voxelgrid filter method
      pcl::VoxelGrid<pcl::PointXYZRGB> vox;
      vox.setInputCloud (intermediate_cloud_ptr);
      vox.setLeafSize (INPUT_CLOUD_VOXEL_FILTER_SIZE,
                       INPUT_CLOUD_VOXEL_FILTER_SIZE,
                       INPUT_CLOUD_VOXEL_FILTER_SIZE);
      vox.filter(*process_cloud_ptr_);
    }
  } /* end namespace detection */
} /* end namespace godel_surface_detection */
