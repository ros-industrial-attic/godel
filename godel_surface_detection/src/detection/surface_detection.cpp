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
#include <coordination//data_coordinator.h>
#include <segmentation/surface_segmentation.h>
#include <godel_param_helpers/godel_param_helpers.h>
#include <tf/transform_datatypes.h>
#include <pluginlib/class_loader.h>
#include <mesher/mesher_plugins.h>

const static int DOWNSAMPLE_NUMBER = 3;

namespace godel_surface_detection
{
  namespace detection
  {
    SurfaceDetection::SurfaceDetection() : full_cloud_ptr_(new CloudRGB()), acquired_clouds_counter_(0)
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
      CloudRGB points;
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

    void SurfaceDetection::add_cloud(CloudRGB& cloud)
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


    void SurfaceDetection::get_surface_clouds(std::vector<CloudRGB::Ptr>& surfaces)
    {
      surfaces.insert(surfaces.end(), surface_clouds_.begin(), surface_clouds_.end());
    }


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


    void SurfaceDetection::get_full_cloud(CloudRGB& cloud)
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
      ROS_INFO_STREAM("Find Surfaces Call");

      // Reset members
      surface_clouds_.clear();
      mesh_markers_.markers.clear();
      meshes_.clear();

      // Ensure cloud ptr is not empty
      if (full_cloud_ptr_->empty())
        return false;

      // Create Processing Cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr process_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>());
      process_cloud_ptr->header = full_cloud_ptr_->header;

      // Subsample full cloud into processing cloud
      for(const auto& pt : full_cloud_ptr_->points)
      {
        int q = rand() % DOWNSAMPLE_NUMBER;
        if (q ==0)
        {
          if(pt.x !=0.0 && pt.y!=0.0 && pt.z !=0.0 && pcl::isFinite(pt))
          {
            process_cloud_ptr->push_back(pt);
          }
        }
      }

      // Segment the part into surface clusters using a "region growing" scheme
      SurfaceSegmentation SS(process_cloud_ptr);
      region_colored_cloud_ptr_ = CloudRGB::Ptr(new CloudRGB());
      SS.computeSegments(region_colored_cloud_ptr_);
      SS.getSurfaceClouds(surface_clouds_);

      // Remove largest cluster if appropriate
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
        surface_clouds_.erase(surface_clouds_.begin() + largest_index);
      }

      pluginlib::ClassLoader<mesher_base::MesherBase> poly_loader("godel_plugins", "mesher_base::MesherBase");
      boost::shared_ptr<mesher_base::MesherBase> mesher;

      try
      {
        mesher = poly_loader.createInstance("mesher_plugins::ConcaveHullMesher");

      }
      catch(pluginlib::PluginlibException& ex)
      {
        ROS_ERROR("The plugin failed to load for some reason. Error: %s", ex.what());
        return false;
      }

      // Compute mesh from point clouds
      for (int i = 0; i < surface_clouds_.size(); i++)
      {
        pcl::PolygonMesh mesh;
        visualization_msgs::Marker marker;
        mesher->init(*surface_clouds_[i]);

        if (mesher->generateMesh(mesh))
        {
          // Create marker from mesh
          mesh_to_marker(mesh, marker);

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
          ROS_INFO_STREAM("Apply concave hull failed");
      }

      return true;
    }
  } /* end namespace detection */
} /* end namespace godel_surface_detection */
