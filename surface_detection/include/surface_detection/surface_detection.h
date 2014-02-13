/*
 * surface_detection.h
 *
 *  Created on: Feb 11, 2014
 *      Author: ros-industrial
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
#include <visualization_msgs/MarkerArray.h>

namespace surface_detection {

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;
typedef pcl::PointCloud<pcl::Normal> Normals;

namespace defaults{

	static const float ACQUISITION_TIME = 5.0f;

	static const int STATISTICAL_OUTLIER_MEAN = 50;
	static const float STATISTICAL_OUTLIER_STDEV_THRESHOLD = 1;
	static const int K_SEARCH = 50;

	static const int REGION_GROWING_MIN_CLUSTER_SIZE=100;
	static const int REGION_GROWING_MAX_CLUSTER_SIZE=100000;
	static const int REGION_GROWING_NEIGHBORS=50;
	static const float REGION_GROWING_SMOOTHNESS_THRESHOLD=(7.0f )/( 180.0f * M_PI);
	static const float REGION_GROWING_CURVATURE_THRESHOLD=1.0f;

	static const float TRIANGULATION_SEARCH_RADIUS = 0.01;
	static const float TRIANGULATION_MU = 2.5f;
	static const int TRIANGULATION_MAX_NEAREST_NEIGHBORS = 100;
	static const float TRIANGULATION_MAX_SURFACE_ANGLE = M_PI/4.0f;
	static const float TRIANGULATION_MIN_ANGLE= M_PI/18.0f;
	static const float TRIANGULATION_MAX_ANGLE = 2.0f *M_PI/3.0f;
	static const bool TRIANGULATION_NORMAL_CONSISTENCY = false;

	static const float VOXEL_LEAF_SIZE = 0.01f;
}

namespace config
{
	static const std::string POINT_COLUD_TOPIC="sensor_point_cloud";

}

class SurfaceDetection {


public:
	SurfaceDetection();
	virtual ~SurfaceDetection();

public:

	void set_acquisition_time(float val);
	bool acquire_data();
	bool find_surfaces();

	// retrieve results
	Cloud::Ptr get_acquired_cloud();
	CloudRGB::Ptr get_region_colored_cloud();
	std::vector<Cloud::Ptr> get_segment_clouds();

protected:


	void point_cloud_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
	bool apply_statistical_filter(const Cloud& in,Cloud& out);
	bool apply_region_growing_segmentation(const Cloud& in,
			const Normals& normals,
			std::vector<pcl::PointIndices>& clusters,CloudRGB& colored_cloud);
	bool apply_normal_estimation(const Cloud &cloud,Normals& normals);
	bool apply_fast_triangulation(const Cloud& in,
			const Normals& normals,
			pcl::PolygonMesh& mesh);

	bool apply_voxel_downsampling(Cloud& cloud);


protected:

	// roscpp members
	ros::Subscriber point_cloud_subs_;

	// pcl members
	pcl::PointCloud<pcl::PointXYZ>::Ptr acquired_cloud_ptr_;
	CloudRGB::Ptr region_colored_cloud_ptr_;
	std::vector<Cloud::Ptr> segment_clouds_;
	visualization_msgs::MarkerArray meshes_;

	// acquisition
	float acquisition_time_;

	// filter and normal estimation
	int acquired_clouds_counter_;
	int meanK_;
	int k_search_;
	float stdv_threshold_;

	// region growing
	int rg_min_cluster_size_;
	int rg_max_cluster_size_;
	int rg_neightbors_;
	float rg_smoothness_threshold_;
	float rg_curvature_threshold_;

	// fast triangulation
	float tr_search_radius_;
	float tr_mu_;
	float tr_max_nearest_neighbors_;
	float tr_max_surface_angle_;
	float tr_min_angle_;
	float tr_max_angle_;
	bool tr_normal_consistency_;

	// voxel downsampling
	float voxel_leafsize_;

};

} /* namespace surface_detection */

#endif /* SURFACE_DETECTION_H_ */
