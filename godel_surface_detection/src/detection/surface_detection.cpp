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

#include <godel_surface_detection/detection/surface_detection.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/mls.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_datatypes.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/surface/ear_clipping.h>
#include <pcl/filters/passthrough.h>
#include <godel_param_helpers/godel_param_helpers.h>

const static double CONCAVE_HULL_ALPHA = 0.1;
const static double PASSTHROUGH_Z_MIN = -0.5;
const static double PASSTHROUGH_Z_MAX = 0.5;

namespace godel_surface_detection
{
namespace detection
{

SurfaceDetection::SurfaceDetection() : full_cloud_ptr_(new Cloud()), acquired_clouds_counter_(0)
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
  params_.ignore_largest_cluster = defaults::IGNORE_LARGEST_CLUSTER;
  params_.occupancy_threshold = defaults::OCCUPANCY_THRESHOLD;
  params_.mls_upsampling_radius = defaults::MLS_UPSAMPLING_RADIUS;
  params_.mls_point_density = defaults::MLS_POINT_DENSITY;
  params_.mls_search_radius = defaults::MLS_SEARCH_RADIUS;
  params_.use_tabletop_seg = defaults::USE_TABLETOP_SEGMENTATION;
  params_.tabletop_seg_distance_threshold = defaults::TABLETOP_SEG_DISTANCE_THRESH;

  srand(time(NULL));
  clear_results();
}

SurfaceDetection::~SurfaceDetection()
{
  // TODO Auto-generated destructor stub
}

bool SurfaceDetection::init()
{
  full_cloud_ptr_->header.frame_id = params_.frame_id;
  acquired_clouds_counter_ = 0;
  return true;
}

void SurfaceDetection::clear_results()
{
  acquired_clouds_counter_ = 0;
  full_cloud_ptr_->clear();
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
         loadParam(nh, params::MARKER_ALPHA, params_.marker_alpha) &&
         loadBoolParam(nh, params::IGNORE_LARGEST_CLUSTER, params_.ignore_largest_cluster);
}

void SurfaceDetection::save_parameters(const std::string& filename)
{
  if (!godel_param_helpers::toFile(filename, params_))
  {
    ROS_WARN_STREAM("Unable to save surface-detection parameters to: " << filename);
  }
}

void SurfaceDetection::mesh_to_marker(const pcl::PolygonMesh& mesh,
                                      visualization_msgs::Marker& marker)
{
  // color value ranges
  static const double color_val_min = 0.5f;
  static const double color_val_max = 1.0f;
  std_msgs::ColorRGBA color;
  color.a = 1;

  // set marker properties
  tf::poseTFToMsg(tf::Transform::getIdentity(), marker.pose);
  marker.scale.x = marker.scale.y = marker.scale.z = 1;
  marker.type = marker.TRIANGLE_LIST;
  marker.action = marker.ADD;

  // create color
  color.r = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  color.g = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  color.b = color_val_min +
            (static_cast<double>(rand()) / static_cast<double>(RAND_MAX)) *
                (color_val_max - color_val_min);
  marker.color = color;

  // filling points
  Cloud points;
  pcl::fromPCLPointCloud2(mesh.cloud, points);
  for (int i = 0; i < mesh.polygons.size(); i++)
  {
    const pcl::Vertices& v = mesh.polygons[i];
    for (int j = 0; j < v.vertices.size(); j++)
    {
      uint32_t index = v.vertices[j];
      geometry_msgs::Point p;
      p.x = points.points[index].x;
      p.y = points.points[index].y;
      p.z = points.points[index].z;
      marker.points.push_back(p);
    }
  }
}

void SurfaceDetection::add_cloud(Cloud& cloud)
{

  (*full_cloud_ptr_) += cloud;
  ROS_INFO_STREAM("Concatenated new cloud to acquired clouds");

  acquired_clouds_counter_++;
  ROS_INFO_STREAM("Surface Detection is currently holding " << acquired_clouds_counter_
                                                            << " point clouds");
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

std::vector<Cloud::Ptr> SurfaceDetection::get_surface_clouds() { return surface_clouds_; }

std::string SurfaceDetection::get_results_summary()
{
  std::stringstream ss;
  if (surface_clouds_.size() > 0)
  {
    ss << "\nNumber of surfaces identified: " << surface_clouds_.size() << "\n";
    for (int i = 0; i < surface_clouds_.size(); i++)
    {
      ss << "\t-segment " << i + 1 << " {points: " << surface_clouds_[i]->size() << "}\n";
    }
  }
  else
  {
    ss << "\nNo surfaces have been found\n";
  }

  return ss.str();
}

void SurfaceDetection::get_full_cloud(Cloud& cloud)
{
  pcl::copyPointCloud(*full_cloud_ptr_, cloud);
}

void SurfaceDetection::get_full_cloud(sensor_msgs::PointCloud2 cloud_msg)
{
  pcl::toROSMsg(*full_cloud_ptr_, cloud_msg);
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
  // main process point cloud
  Cloud::Ptr process_cloud_ptr = Cloud::Ptr(new Cloud());

  if (full_cloud_ptr_->empty())
  {
    return false;
  }
  else
  {
    pcl::copyPointCloud(*full_cloud_ptr_, *process_cloud_ptr);
  }

  // reset members
  region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
  surface_clouds_.clear();
  mesh_markers_.markers.clear();
  meshes_.clear();

  // variables to hold intermediate results
  Normals::Ptr normals(new Normals());
  std::vector<pcl::PointIndices> clusters_indices;
  std::vector<Normals::Ptr> segment_normals;

  // Pass through filter the data to constrain it to our ROI
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(process_cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(PASSTHROUGH_Z_MIN, PASSTHROUGH_Z_MAX);
  pass.filter(*process_cloud_ptr);

  ROS_INFO_STREAM("Surface detection processing a cloud containing " << process_cloud_ptr->size()
                                                                     << " points");

  if (apply_voxel_downsampling(*process_cloud_ptr))
  {
    ROS_INFO_STREAM(
        "Voxel downsampling succeeded, downsampled cloud size: " << process_cloud_ptr->size());
  }
  else
  {
    ROS_WARN_STREAM("Voxel downsampling failed, cloud size :" << process_cloud_ptr->size());
  }

  if (params_.use_tabletop_seg)
  {
    int count = process_cloud_ptr->size();
    if (apply_tabletop_segmentation(*process_cloud_ptr, *process_cloud_ptr))
    {
      ROS_INFO_STREAM("Tabletop segmentation successfully applied, new point count: "
                      << process_cloud_ptr->size() << ", old point cloud: " << count);
    }
    else
    {
      ROS_WARN_STREAM("Tabletop segmentation failed, ignoring results");
    }
  }
  else
  {
    ROS_WARN_STREAM("Tabletop segmentation skipped");
  }

  // apply statistical filter
  ROS_INFO_STREAM("Statistical filter started with " << process_cloud_ptr->size() << " points");
  if (apply_statistical_filter(*process_cloud_ptr, *process_cloud_ptr))
  {
    ROS_INFO_STREAM("Statistical filter completed with " << process_cloud_ptr->size() << " points");
  }
  else
  {
    ROS_ERROR_STREAM("Statistical filter failed");
    return false;
  }

  // estimate normals
  if (apply_normal_estimation(*process_cloud_ptr, *normals))
  {
    ROS_INFO_STREAM("Normal estimation succeeded");
  }
  else
  {
    ROS_ERROR_STREAM("Normal estimation failed");
    return false;
  }

  // applying region growing segmentation
  Cloud::Ptr ungrouped_cloud_ptr = Cloud::Ptr(new Cloud()); // cloud for storing outliers
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
  if (apply_region_growing_segmentation(*process_cloud_ptr, *normals, clusters_indices,
                                        *region_colored_cloud_ptr_))
  {

    ROS_INFO_STREAM("Region growing succeeded");
    ROS_INFO_STREAM("Total surface clusters found: " << clusters_indices.size());

    // saving clusters that meet required point count
    for (int i = 0; i < clusters_indices.size(); i++)
    {

      Cloud::Ptr segment_cloud_ptr = Cloud::Ptr(new Cloud());
      pcl::PointIndices& indices = clusters_indices[i];
      if (indices.indices.size() == 0)
      {
        continue;
      }

      if (indices.indices.size() >= params_.rg_min_cluster_size)
      {
        inliers_ptr->indices.insert(inliers_ptr->indices.end(), indices.indices.begin(),
                                    indices.indices.end());

        pcl::copyPointCloud(*process_cloud_ptr, indices, *segment_cloud_ptr);
        surface_clouds_.push_back(segment_cloud_ptr);
      }
    }

    // saving all ungrouped points
    extract.setInputCloud(process_cloud_ptr);
    extract.setIndices(inliers_ptr);
    extract.setNegative(true);
    extract.filter(*ungrouped_cloud_ptr);

    ROS_INFO_STREAM("Total ungrouped points: " << ungrouped_cloud_ptr->size());
    ROS_INFO_STREAM("Valid surface clusters kept :" << surface_clouds_.size());
  }
  else
  {
    ROS_ERROR_STREAM("Region growing failed");
    return false;
  }

  // applying sac plane segmentation to reintegrate ungrouped points
  for (int i = 0;
       (i < surface_clouds_.size()) && ungrouped_cloud_ptr->size() > 0 && params_.pa_enabled; i++)
  {

    Cloud::Ptr segment_cloud_ptr = surface_clouds_[i];
    int count = segment_cloud_ptr->size();

    if (apply_plane_projection_refinement(*ungrouped_cloud_ptr, *segment_cloud_ptr,
                                          *segment_cloud_ptr))
    {

      ROS_INFO_STREAM("Plane approximation refinement for cluster "
                      << i << " completed with [ star:  " << count
                      << ", end: " << segment_cloud_ptr->size() << " ] points");
    }
    else
    {
      ROS_WARN_STREAM("Plane approximation refinement for cluster "
                      << i << " yielded no matches [ star:  " << count
                      << ", end: " << segment_cloud_ptr->size() << " ] points");
    }
  }

  // applying mls surface smoothing
  for (int i = 0; i < surface_clouds_.size(); i++)
  {

    Normals::Ptr segment_normal_ptr = Normals::Ptr(new Normals());
    Cloud::Ptr segment_cloud_ptr = surface_clouds_[i];
    int count = segment_cloud_ptr->size();
    if (params_.mls_point_density > 0 &&
        apply_mls_surface_smoothing(*segment_cloud_ptr, *segment_cloud_ptr, *segment_normal_ptr))
    {
      ROS_INFO_STREAM("Moving least squares smoothing for cluster "
                      << i << " completed with [ star:  " << count
                      << ", end: " << segment_cloud_ptr->size() << " ] points");
    }
    else
    {
      ROS_WARN_STREAM("Moving least squares smoothing "
                      << (params_.mls_point_density > 0 ? "failed" : "disabled") << " for cluster "
                      << i << ", estimating normals");
      apply_normal_estimation(*segment_cloud_ptr, *segment_normal_ptr);
    }

    segment_cloud_ptr->header.frame_id = params_.frame_id;
    segment_normals.push_back(segment_normal_ptr);
  }

  ROS_INFO_STREAM("Selected surface clusters (> " << params_.rg_min_cluster_size
                                                  << " points ) found: " << surface_clouds_.size());

  if (params_.ignore_largest_cluster && surface_clouds_.size() > 1)
  {
    int largest_index = 0;
    int largest_size = 0;
    for (int i = 0; i < surface_clouds_.size(); i++)
    {
      if (surface_clouds_[i]->points.size() > largest_size)
      {
        largest_size = surface_clouds_[i]->points.size();
        largest_index = i;
      }
    }

    ROS_INFO_STREAM("Removing larges cluster from results: cluster index [ "
                    << largest_index << " ], cluster size [ " << largest_size << " ]");
    surface_clouds_.erase(surface_clouds_.begin() + largest_index);
    segment_normals.erase(segment_normals.begin() + largest_index);
  }

  // applying fast triangulation

  ROS_INFO_STREAM("Triangulation of surfaces started");
  for (int i = 0; i < surface_clouds_.size(); i++)
  {
    pcl::PolygonMesh mesh;
    visualization_msgs::Marker marker;

    if (!apply_planar_reprojection(*surface_clouds_[i], *surface_clouds_[i]))
    {
      continue;
    }

    if (apply_concave_hull(*surface_clouds_[i], mesh))
    {
      mesh_to_marker(mesh, marker);

      // saving other properties
      marker.header.frame_id = mesh.header.frame_id = surface_clouds_[i]->header.frame_id;
      marker.id = i;
      marker.color.a = params_.marker_alpha;
      mesh_markers_.markers.push_back(marker);
      meshes_.push_back(mesh);

      ROS_INFO_STREAM("Triangulation for surface " << i << " completed");
    }
    else
    {
      ROS_WARN_STREAM("Triangulation for surface " << i << " failed");
    }
  }

  ROS_INFO_STREAM("Triangulation of surfaces completed");

  return true;
}

bool SurfaceDetection::apply_statistical_filter(const Cloud& in, Cloud& out)
{
  if (params_.meanK == 0)
  {
    // skip
    return true;
  }

  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> filter;
  Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(in);
  filter.setInputCloud(cloud_ptr);
  filter.setMeanK(params_.meanK);
  filter.setStddevMulThresh(params_.stdv_threshold);
  filter.filter(out);

  return !out.empty();
}

bool SurfaceDetection::apply_normal_estimation(const Cloud& cloud, Normals& normals)
{
  Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(cloud);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setViewPoint(0, 0, 5.0f);
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_ptr);
  normal_estimator.setKSearch(params_.k_search);
  // normal_estimator.setRadiusSearch(0.002f);
  normal_estimator.compute(normals);

  return !normals.empty();
}

bool SurfaceDetection::apply_region_growing_segmentation(const Cloud& in, const Normals& normals,
                                                         std::vector<pcl::PointIndices>& clusters,
                                                         CloudRGB& colored_cloud)
{
  Cloud::ConstPtr cloud_ptr = boost::make_shared<Cloud>(in);
  const Normals::Ptr normals_ptr = boost::make_shared<Normals>(normals);
  pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> rg;
  rg.setMinClusterSize(params_.rg_min_cluster_size);
  rg.setMaxClusterSize(params_.rg_max_cluster_size);
  rg.setSearchMethod(tree);
  rg.setNumberOfNeighbours(params_.rg_neightbors);
  rg.setInputCloud(cloud_ptr);
  rg.setInputNormals(normals_ptr);
  rg.setSmoothnessThreshold(params_.rg_smoothness_threshold);
  rg.setCurvatureThreshold(params_.rg_curvature_threshold);
  rg.extract(clusters);

  if (rg.getColoredCloud() != 0)
  {
    pcl::copyPointCloud(*rg.getColoredCloud(), colored_cloud);
  }

  return clusters.size() > 0;
}

bool SurfaceDetection::apply_plane_projection_refinement(const Cloud& candidate_outliers,
                                                         const Cloud& surface_cluster,
                                                         Cloud& projected_cluster)
{
  pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr plane_inliers_ptr(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // setting segmentation options
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.pa_seg_max_iterations);
  seg.setDistanceThreshold(params_.pa_seg_dist_threshold);

  // finding coefficients
  seg.setInputCloud(surface_cluster.makeShared());
  seg.segment(*plane_inliers_ptr, *coeff_ptr);

  // plane projection and proximity search
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> model_plane(candidate_outliers.makeShared());
  Eigen::VectorXf model_coeff(4);

  if (plane_inliers_ptr->indices.size() > 0)
  {
    // projecting to plane
    pcl::ProjectInliers<pcl::PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(surface_cluster.makeShared());
    proj.setModelCoefficients(coeff_ptr);
    proj.filter(projected_cluster);

    // adding near points from outliers
    std::vector<int> final_inliers;
    Cloud::Ptr inlier_points_ptr = Cloud::Ptr(new Cloud());
    model_plane.setInputCloud(candidate_outliers.makeShared());
    model_coeff << coeff_ptr->values[0], coeff_ptr->values[1], coeff_ptr->values[2],
        coeff_ptr->values[3];
    model_plane.selectWithinDistance(model_coeff, params_.pa_sac_plane_distance, final_inliers);
    pcl::copyPointCloud(candidate_outliers, final_inliers, *inlier_points_ptr);

    // checking distances to main cluster
    if (apply_kdtree_radius_search(*inlier_points_ptr, surface_cluster, params_.pa_kdtree_radius,
                                   *inlier_points_ptr))
    {
      // adding to output cloud
      projected_cluster += *inlier_points_ptr;
    }
  }
  else
  {
    return false;
  }

  return true;
}

bool SurfaceDetection::apply_kdtree_radius_search(const Cloud& query_points,
                                                  const Cloud& search_points, double radius,
                                                  Cloud& close_points)
{
  pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
  std::vector<int> all_indices;
  std::vector<int> near_indices;
  std::vector<float> near_distances;

  kdtree.setInputCloud(search_points.makeShared());
  for (int i = 0; i < query_points.size(); i++)
  {
    const pcl::PointXYZ& p = query_points[i];
    near_indices.clear();
    near_distances.clear();
    if (kdtree.radiusSearch(p, radius, near_indices, near_distances) > 0)
    {
      all_indices.push_back(i);
    }
  }

  if (all_indices.size() > 0)
  {
    pcl::copyPointCloud(query_points, all_indices, close_points);
    return true;
  }
  else
  {
    return false;
  }
}

bool SurfaceDetection::apply_voxel_downsampling(Cloud& cloud)
{
  if (params_.voxel_leafsize > 0.000001f)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    double min, max;
    vg.getFilterLimits(min, max);
    ROS_INFO_STREAM("Voxel downsampling with filter limits min: " << min << ", max: " << max);
    vg.setInputCloud(cloud.makeShared());
    vg.setLeafSize(params_.voxel_leafsize, params_.voxel_leafsize, params_.voxel_leafsize);
    vg.filter(cloud);
  }
  else
  {
    ROS_WARN_STREAM("Voxel downsampling leaf too close to 0, skipping.");
  }
  return true;
}

bool SurfaceDetection::apply_mls_surface_smoothing(const Cloud& cloud_in, Cloud& cloud_out,
                                                   Normals& normals)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointNormal> mls_points;
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setInputCloud(boost::make_shared<Cloud>(cloud_in));
  mls.setComputeNormals(true);
  mls.setPolynomialFit(true);

  mls.setUpsamplingMethod(
      pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
  mls.setUpsamplingRadius(params_.mls_upsampling_radius);
  mls.setPointDensity(params_.mls_point_density);

  mls.setSearchMethod(tree);
  mls.setSearchRadius(params_.mls_search_radius);
  mls.process(mls_points);

  bool succeeded = mls_points.size() > 0;
  if (succeeded)
  {
    cloud_out.clear();
    normals.clear();
    pcl::copyPointCloud(mls_points, cloud_out);
    pcl::copyPointCloud(mls_points, normals);
  }
  return succeeded;
}

bool SurfaceDetection::apply_tabletop_segmentation(const Cloud& cloud_in, Cloud& cloud_out)
{
  pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.pa_seg_max_iterations);
  seg.setDistanceThreshold(params_.tabletop_seg_distance_threshold);
  seg.setInputCloud(boost::make_shared<Cloud>(cloud_in));
  seg.segment(*inliers_ptr, *coeff_ptr);

  bool succeeded = inliers_ptr->indices.size() > 0;
  if (succeeded)
  {
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(boost::make_shared<Cloud>(cloud_in));
    extract.setIndices(inliers_ptr);
    extract.setNegative(true);
    extract.filter(cloud_out);
  }

  return succeeded;
}

bool SurfaceDetection::apply_planar_reprojection(const Cloud& in, Cloud& out)
{
  pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr plane_inliers_ptr(new pcl::PointIndices());
  pcl::SACSegmentation<pcl::PointXYZ> seg;

  // setting segmentation options
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(params_.pa_seg_max_iterations);
  seg.setDistanceThreshold(params_.tabletop_seg_distance_threshold);

  // finding coefficients
  seg.setInputCloud(in.makeShared());
  seg.segment(*plane_inliers_ptr, *coeff_ptr);

  if (plane_inliers_ptr->indices.size() == 0)
  {
    ROS_WARN_STREAM(__FUNCTION__ << ": Could not segment out plane");
    return false;
  }

  // If successful, extract points relevant to the plane
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setInputCloud(in.makeShared());
  extract.setIndices(plane_inliers_ptr);
  extract.setNegative(false);
  extract.filter(out);

  // projecting to plane
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(out.makeShared());
  proj.setModelCoefficients(coeff_ptr);
  proj.filter(out);
  return true;
}

bool SurfaceDetection::apply_concave_hull(const Cloud& in, pcl::PolygonMesh& mesh)
{
  pcl::PolygonMesh::Ptr mesh_ptr(new pcl::PolygonMesh);
  pcl::ConcaveHull<pcl::PointXYZ> chull;
  chull.setInputCloud(in.makeShared());
  chull.setAlpha(CONCAVE_HULL_ALPHA);
  chull.reconstruct(*mesh_ptr);

  // Given a complex concave hull polygon, seperate it into triangles
  pcl::EarClipping clipping;
  clipping.setInputMesh(mesh_ptr);
  clipping.process(mesh);

  return mesh.polygons.size() > 0;
}
}
} /* namespace godel_surface_detection */
