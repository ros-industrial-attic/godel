# acquisition
string frame_id

# filter and normal estimation
int32 meanK
int32 k_search
float64 stdv_threshold

# region growing
int32 rg_min_cluster_size
int32 rg_max_cluster_size
int32 rg_neightbors
float64 rg_smoothness_threshold
float64 rg_curvature_threshold

# fast triangulation
float64 tr_search_radius
float64 tr_mu
int32 tr_max_nearest_neighbors
float64 tr_max_surface_angle
float64 tr_min_angle
float64 tr_max_angle
bool tr_normal_consistency

# plane approximation refinement: Assumes planar surfaces in order to identify additional inlier points
bool pa_enabled
int32 pa_seg_max_iterations
float64 pa_seg_dist_threshold
float64 pa_sac_plane_distance
float64 pa_kdtree_radius

# voxel downsampling
float64 voxel_leafsize

# octomap occupancy
bool use_octomap
float64 occupancy_threshold

# moving least square smoothing
float64 mls_upsampling_radius
float64 mls_search_radius
int32 mls_point_density

# tabletop segmentation
bool use_tabletop_seg
float64 tabletop_seg_distance_threshold

# options
float64 marker_alpha
bool ignore_largest_cluster 
