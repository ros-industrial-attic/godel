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

#ifndef SURFACE_DETECTION_H_
#define SURFACE_DETECTION_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <godel_msgs/SurfaceDetectionParameters.h>
#include <visualization_msgs/MarkerArray.h>

namespace godel_surface_detection
{
namespace detection
{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;
typedef pcl::PointCloud<pcl::Normal> Normals;

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
static const bool IGNORE_LARGEST_CLUSTER = false;
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
static const std::string IGNORE_LARGEST_CLUSTER = "ignore_largest_cluster";
}

class SurfaceDetection
{

public:
  SurfaceDetection();
  virtual ~SurfaceDetection();

public:
  bool init();

  bool load_parameters(const std::string& filename);
  void save_parameters(const std::string& filename);

  bool find_surfaces();
  std::string get_results_summary();

  static void mesh_to_marker(const pcl::PolygonMesh& mesh, visualization_msgs::Marker& marker);

  // adds point cloud to the occupancy grid, it performs no frame transformation
  void add_cloud(Cloud& cloud);
  int get_acquired_clouds_count();

  void clear_results();

  // retrieve results
  visualization_msgs::MarkerArray get_surface_markers();
  void get_meshes(std::vector<pcl::PolygonMesh>& meshes);
  std::vector<Cloud::Ptr> get_surface_clouds();
  void get_full_cloud(Cloud& cloud);
  void get_full_cloud(sensor_msgs::PointCloud2 cloud_msg);
  void get_region_colored_cloud(CloudRGB& cloud);
  void get_region_colored_cloud(sensor_msgs::PointCloud2& cloud_msg);

protected:
  bool apply_statistical_filter(const Cloud& in, Cloud& out);
  bool apply_region_growing_segmentation(const Cloud& in, const Normals& normals,
                                         std::vector<pcl::PointIndices>& clusters,
                                         CloudRGB& colored_cloud);
  bool apply_plane_projection_refinement(const Cloud& candidate_outliers,
                                         const Cloud& surface_cluster, Cloud& projected_cluster);

  bool apply_normal_estimation(const Cloud& cloud, Normals& normals);

  bool apply_kdtree_radius_search(const Cloud& query_points, const Cloud& search_points,
                                  double radius, Cloud& close_points);

  bool apply_voxel_downsampling(Cloud& cloud);

  bool apply_mls_surface_smoothing(const Cloud& cloud_in, Cloud& cloud_out, Normals& normals);

  bool apply_tabletop_segmentation(const Cloud& cloud_in, Cloud& cloud_out);

  /* @brief Finds the best fit plane for the input cloud, reprojects points onto the plane,
   * and then calculates the concave hull and triangulates a mesh.
   */
  bool apply_planar_reprojection(const Cloud& in, Cloud& out);
  bool apply_concave_hull(const Cloud& in, pcl::PolygonMesh& mesh);

public:
  // parameters
  godel_msgs::SurfaceDetectionParameters params_;

protected:
  // roscpp members
  ros::Subscriber point_cloud_subs_;

  // pcl members
  pcl::PointCloud<pcl::PointXYZ>::Ptr full_cloud_ptr_;
  CloudRGB::Ptr region_colored_cloud_ptr_;
  std::vector<Cloud::Ptr> surface_clouds_;
  visualization_msgs::MarkerArray mesh_markers_;
  std::vector<pcl::PolygonMesh> meshes_;

  // counter
  int acquired_clouds_counter_;
};
}
} /* namespace godel_surface_detection */

#endif /* SURFACE_DETECTION_H_ */
