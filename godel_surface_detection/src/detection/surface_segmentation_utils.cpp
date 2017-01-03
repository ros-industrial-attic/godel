#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/detection/surface_segmentation.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>

const static double CONCAVE_HULL_ALPHA = 0.1;

namespace godel_surface_detection
{
  namespace detection
  {
    bool SurfaceDetection::apply_statistical_filter(const CloudRGB& in, CloudRGB& out)
    {
      if (params_.meanK == 0)
      {
        // skip
        return true;
      }

      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> filter;
      CloudRGB::ConstPtr cloud_ptr = boost::make_shared<CloudRGB>(in);
      filter.setInputCloud(cloud_ptr);
      filter.setMeanK(params_.meanK);
      filter.setStddevMulThresh(params_.stdv_threshold);
      filter.filter(out);

      return !out.empty();
    }

    bool SurfaceDetection::apply_normal_estimation(const CloudRGB& cloud, Normals& normals)
    {
      CloudRGB::ConstPtr cloud_ptr = boost::make_shared<CloudRGB>(cloud);
      pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
      normal_estimator.setViewPoint(0, 0, 5.0f);
      normal_estimator.setSearchMethod(tree);
      normal_estimator.setInputCloud(cloud_ptr);
      normal_estimator.setKSearch(params_.k_search);
      // normal_estimator.setRadiusSearch(0.002f);
      normal_estimator.compute(normals);

      return !normals.empty();
    }

    bool SurfaceDetection::apply_region_growing_segmentation(const CloudRGB& in, const Normals& normals,
                                                             std::vector<pcl::PointIndices>& clusters,
                                                             CloudRGB& colored_cloud)
    {
      CloudRGB::ConstPtr cloud_ptr = boost::make_shared<CloudRGB>(in);
      const Normals::Ptr normals_ptr = boost::make_shared<Normals>(normals);
      pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
      pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> rg;
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

    bool SurfaceDetection::apply_plane_projection_refinement(const CloudRGB& candidate_outliers,
                                                             const CloudRGB& surface_cluster,
                                                             CloudRGB& projected_cluster)
    {
      pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr plane_inliers_ptr(new pcl::PointIndices());
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;

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
      pcl::SampleConsensusModelPlane<pcl::PointXYZRGB> model_plane(candidate_outliers.makeShared());
      Eigen::VectorXf model_coeff(4);

      if (plane_inliers_ptr->indices.size() > 0)
      {
        // projecting to plane
        pcl::ProjectInliers<pcl::PointXYZRGB> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(surface_cluster.makeShared());
        proj.setModelCoefficients(coeff_ptr);
        proj.filter(projected_cluster);

        // adding near points from outliers
        std::vector<int> final_inliers;
        CloudRGB::Ptr inlier_points_ptr = CloudRGB::Ptr(new CloudRGB());
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

    bool SurfaceDetection::apply_kdtree_radius_search(const CloudRGB& query_points,
                                                      const CloudRGB& search_points, double radius,
                                                      CloudRGB& close_points)
    {
      pcl::KdTreeFLANN<pcl::PointXYZRGB> kdtree;
      std::vector<int> all_indices;
      std::vector<int> near_indices;
      std::vector<float> near_distances;

      kdtree.setInputCloud(search_points.makeShared());
      for (int i = 0; i < query_points.size(); i++)
      {
        const pcl::PointXYZRGB& p = query_points[i];
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

    bool SurfaceDetection::apply_voxel_downsampling(CloudRGB& cloud)
    {
      if (params_.voxel_leafsize > 0.000001f)
      {
        pcl::VoxelGrid<pcl::PointXYZRGB> vg;
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

    bool SurfaceDetection::apply_mls_surface_smoothing(const CloudRGB& cloud_in, CloudRGB& cloud_out,
                                                       Normals& normals)
    {
      return false;
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
      pcl::PointCloud<pcl::PointNormal> mls_points;
      pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
      mls.setInputCloud(boost::make_shared<CloudRGB>(cloud_in));
      mls.setComputeNormals(true);
      mls.setPolynomialFit(true);

      mls.setUpsamplingMethod(
          pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal>::RANDOM_UNIFORM_DENSITY);
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

    bool SurfaceDetection::apply_tabletop_segmentation(const CloudRGB& cloud_in, CloudRGB& cloud_out)
    {
      pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices());
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_PLANE);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setMaxIterations(params_.pa_seg_max_iterations);
      seg.setDistanceThreshold(params_.tabletop_seg_distance_threshold);
      seg.setInputCloud(boost::make_shared<CloudRGB>(cloud_in));
      seg.segment(*inliers_ptr, *coeff_ptr);

      bool succeeded = inliers_ptr->indices.size() > 0;
      if (succeeded)
      {
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(boost::make_shared<CloudRGB>(cloud_in));
        extract.setIndices(inliers_ptr);
        extract.setNegative(true);
        extract.filter(cloud_out);
      }

      return succeeded;
    }

    bool SurfaceDetection::apply_planar_reprojection(const CloudRGB& in, CloudRGB& out)
    {
      pcl::ModelCoefficients::Ptr coeff_ptr(new pcl::ModelCoefficients());
      pcl::PointIndices::Ptr plane_inliers_ptr(new pcl::PointIndices());
      pcl::SACSegmentation<pcl::PointXYZRGB> seg;

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

      // HACK: Return false if plane is rotated too far from Z axis
      Eigen::Vector3d plane_normal ((*coeff_ptr).values[0], (*coeff_ptr).values[1], (*coeff_ptr).values[2]);
      Eigen::Vector3d z_dir (0, 0, 1);

      double z_angle = std::abs(std::acos( plane_normal.normalized().dot(z_dir) ));
      const double max_angle = M_PI / 3.0;
      if (z_angle > max_angle && z_angle < (M_PI - max_angle))
      {
        ROS_WARN("Surface normal not close enough to Z: %f vs max of %f", z_angle, max_angle);
        return false;
      }

      // If successful, extract points relevant to the plane
      pcl::ExtractIndices<pcl::PointXYZRGB> extract;
      extract.setInputCloud(in.makeShared());
      extract.setIndices(plane_inliers_ptr);
      extract.setNegative(false);
      extract.filter(out);

      // projecting to plane
      pcl::ProjectInliers<pcl::PointXYZRGB> proj;
      proj.setModelType(pcl::SACMODEL_PLANE);
      proj.setInputCloud(out.makeShared());
      proj.setModelCoefficients(coeff_ptr);
      proj.filter(out);
      return true;
    }

    bool SurfaceDetection::apply_concave_hull(const CloudRGB& in, pcl::PolygonMesh& mesh)
    {
      pcl::PolygonMesh::Ptr mesh_ptr(new pcl::PolygonMesh);
      pcl::ConcaveHull<pcl::PointXYZRGB> chull;
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
}
