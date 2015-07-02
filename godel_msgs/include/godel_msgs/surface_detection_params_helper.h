#ifndef SURFACE_DETECTION_PARAMS_HELPER
#define SURFACE_DETECTION_PARAMS_HELPER

#include <param_helpers/param_interfaces.h>
#include <godel_msgs/SurfaceDetectionParameters.h>

namespace param_helpers 
{

template<>
struct ParamInterfaces<godel_msgs::SurfaceDetectionParameters>
{
  template<typename Setter>
  static void save(Setter& setter, const godel_msgs::SurfaceDetectionParameters& params)
  {
    //  acquisition
    setter.set("frame_id", params.frame_id);

    //  filter and normal estimation
    setter.set("meanK", params.meanK);
    setter.set("k_search", params.k_search);
    setter.set("stdv_threshold", params.stdv_threshold);

    //  region growing
    setter.set("rg_min_cluster_size", params.rg_min_cluster_size);
    setter.set("rg_max_cluster_size", params.rg_max_cluster_size);
    setter.set("rg_neightbors", params.rg_neightbors);
    setter.set("rg_smoothness_threshold", params.rg_smoothness_threshold);
    setter.set("rg_curvature_threshold", params.rg_curvature_threshold);

    //  fast triangulation
    setter.set("tr_search_radius", params.tr_search_radius);
    setter.set("tr_mu", params.tr_mu);
    setter.set("tr_max_nearest_neighbors", params.tr_max_nearest_neighbors);
    setter.set("tr_max_surface_angle", params.tr_max_surface_angle);
    setter.set("tr_min_angle", params.tr_min_angle);
    setter.set("tr_max_angle", params.tr_max_angle);
    bool b0 = params.tr_normal_consistency; setter.set("tr_normal_consistency", b0);

    //  plane approximation refinement: Assumes planar surfaces in order to identify additional inlier points
    bool b1 = params.pa_enabled; setter.set("pa_enabled", b1);
    setter.set("pa_seg_max_iterations", params.pa_seg_max_iterations);
    setter.set("pa_seg_dist_threshold", params.pa_seg_dist_threshold);
    setter.set("pa_sac_plane_distance", params.pa_sac_plane_distance);
    setter.set("pa_kdtree_radius", params.pa_kdtree_radius);

    //  voxel downsampling
    setter.set("voxel_leafsize", params.voxel_leafsize);

    //  octomap occupancy
    bool b2 = params.use_octomap; setter.set("use_octomap", b2);
    setter.set("occupancy_threshold", params.occupancy_threshold);

    //  moving least square smoothing
    setter.set("mls_upsampling_radius", params.mls_upsampling_radius);
    setter.set("mls_search_radius", params.mls_search_radius);
    setter.set("mls_point_density", params.mls_point_density);

    //  tabletop segmentation
    bool b3 = params.use_tabletop_seg; setter.set("use_tabletop_seg", b3);
    setter.set("tabletop_seg_distance_threshold", params.tabletop_seg_distance_threshold);

    //  options
    setter.set("marker_alpha", params.marker_alpha);
    bool b4 = params.ignore_largest_cluster; setter.set("ignore_largest_cluster", b4);
  }

  template<typename Getter>
  static void load(Getter& getter, godel_msgs::SurfaceDetectionParameters& params)
  {
    //  acquisition
    getter.get("frame_id", params.frame_id);

    //  filter and normal estimation
    getter.get("meanK", params.meanK);
    getter.get("k_search", params.k_search);
    getter.get("stdv_threshold", params.stdv_threshold);

    //  region growing
    getter.get("rg_min_cluster_size", params.rg_min_cluster_size);
    getter.get("rg_max_cluster_size", params.rg_max_cluster_size);
    getter.get("rg_neightbors", params.rg_neightbors);
    getter.get("rg_smoothness_threshold", params.rg_smoothness_threshold);
    getter.get("rg_curvature_threshold", params.rg_curvature_threshold);

    //  fast triangulation
    getter.get("tr_search_radius", params.tr_search_radius);
    getter.get("tr_mu", params.tr_mu);
    getter.get("tr_max_nearest_neighbors", params.tr_max_nearest_neighbors);
    getter.get("tr_max_surface_angle", params.tr_max_surface_angle);
    getter.get("tr_min_angle", params.tr_min_angle);
    getter.get("tr_max_angle", params.tr_max_angle);
    bool b0; getter.get("tr_normal_consistency", b0); params.tr_normal_consistency = b0;

    //  plane approximation refinement: Assumes planar surfaces in order to identify additional inlier points
    bool b1; getter.get("pa_enabled", b1); params.pa_enabled = b1;
    getter.get("pa_seg_max_iterations", params.pa_seg_max_iterations);
    getter.get("pa_seg_dist_threshold", params.pa_seg_dist_threshold);
    getter.get("pa_sac_plane_distance", params.pa_sac_plane_distance);
    getter.get("pa_kdtree_radius", params.pa_kdtree_radius);

    //  voxel downsampling
    getter.get("voxel_leafsize", params.voxel_leafsize);

    //  octomap occupancy
    bool b2; getter.get("use_octomap", b2); params.use_octomap = b2;
    getter.get("occupancy_threshold", params.occupancy_threshold);

    //  moving least square smoothing
    getter.get("mls_upsampling_radius", params.mls_upsampling_radius);
    getter.get("mls_search_radius", params.mls_search_radius);
    getter.get("mls_point_density", params.mls_point_density);

    //  tabletop segmentation
    bool b3; getter.get("use_tabletop_seg", b3); params.use_tabletop_seg = b3;
    getter.get("tabletop_seg_distance_threshold", params.tabletop_seg_distance_threshold);

    //  options
    getter.get("marker_alpha", params.marker_alpha);
    bool b4; getter.get("ignore_largest_cluster", b4); params.ignore_largest_cluster = b4;
  }
};



}

#endif
