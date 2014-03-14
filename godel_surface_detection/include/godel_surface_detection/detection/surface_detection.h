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
#include <visualization_msgs/MarkerArray.h>

namespace godel_surface_detection { namespace detection{

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZRGB> CloudRGB;
typedef pcl::PointCloud<pcl::Normal> Normals;

namespace defaults{

	static const double ACQUISITION_TIME = 5.0f;
	static std::string FRAME_ID = "world_frame";

	static const int STATISTICAL_OUTLIER_MEAN = 50;
	static const double STATISTICAL_OUTLIER_STDEV_THRESHOLD = 1;
	static const int K_SEARCH = 50;

	static const int REGION_GROWING_MIN_CLUSTER_SIZE=100;
	static const int REGION_GROWING_MAX_CLUSTER_SIZE=100000;
	static const int REGION_GROWING_NEIGHBORS=50;
	static const double REGION_GROWING_SMOOTHNESS_THRESHOLD=(M_PI/180.0f) * 7.0f;
	static const double REGION_GROWING_CURVATURE_THRESHOLD=1.0f;

	static const double TRIANGULATION_SEARCH_RADIUS = 0.01;
	static const double TRIANGULATION_MU = 2.5f;
	static const int TRIANGULATION_MAX_NEAREST_NEIGHBORS = 100;
	static const double TRIANGULATION_MAX_SURFACE_ANGLE = M_PI/4.0f;
	static const double TRIANGULATION_MIN_ANGLE= M_PI/18.0f;
	static const double TRIANGULATION_MAX_ANGLE = 2.0f *M_PI/3.0f;
	static const bool TRIANGULATION_NORMAL_CONSISTENCY = false;

	static const double VOXEL_LEAF_SIZE = 0.01f;

	static const double MARKER_ALPHA = 1.0f;

	static const bool IGNORE_LARGEST_CLUSTER = false;
}

namespace config
{
	static const std::string POINT_COLUD_TOPIC="sensor_point_cloud";
}

namespace params
{
	static const std::string PARAMETER_NS = "surface_detection";

	static const std::string ACQUISITION_TIME = "acquisition_time";
	static const std::string FRAME_ID = "frame_id";

	static const std::string STOUTLIER_MEAN = "stout_mean";
	static const std::string STOUTLIER_STDEV_THRESHOLD = "stout_stdev_threshold";
	static const std::string K_SEARCH = "k_search";

	static const std::string REGION_GROWING_MIN_CLUSTER_SIZE="rg_min_cluster_size";
	static const std::string REGION_GROWING_MAX_CLUSTER_SIZE="rg_max_cluster_size";
	static const std::string REGION_GROWING_NEIGHBORS="rg_neighbors";
	static const std::string REGION_GROWING_SMOOTHNESS_THRESHOLD="rg_smoothness_threshold";
	static const std::string REGION_GROWING_CURVATURE_THRESHOLD="rg_curvature_threshold";

	static const std::string TRIANGULATION_SEARCH_RADIUS = "tr_search_radius";
	static const std::string TRIANGULATION_MU = "tr_mu";
	static const std::string TRIANGULATION_MAX_NEAREST_NEIGHBORS = "tr_nearest_neighbors";
	static const std::string TRIANGULATION_MAX_SURFACE_ANGLE ="tr_max_surface_angle";
	static const std::string TRIANGULATION_MIN_ANGLE= "tr_min_angle";
	static const std::string TRIANGULATION_MAX_ANGLE = "tr_max_angle";
	static const std::string TRIANGULATION_NORMAL_CONSISTENCY = "tr_normal_consistency";

	static const std::string VOXEL_LEAF_SIZE = "voxel_leaf";

	static const std::string MARKER_ALPHA = "marker_alpha";
	static const std::string IGNORE_LARGEST_CLUSTER = "ignore_largest_cluster";

}

class SurfaceDetection {


public:
	SurfaceDetection();
	virtual ~SurfaceDetection();

public:

	bool init(std::string node_ns = "");
	void set_acquisition_time(double val);
	bool acquire_data();
	bool find_surfaces();
	std::string get_results_summary();

	static void mesh_to_marker(const pcl::PolygonMesh &mesh,
			visualization_msgs::Marker &marker);

	// retrieve results
	visualization_msgs::MarkerArray get_surface_markers();
	std::vector<Cloud::Ptr> get_surface_clouds();
	void get_acquired_cloud(Cloud& cloud);
	void get_acquired_cloud(sensor_msgs::PointCloud2 cloud_msg);
	void get_region_colored_cloud(CloudRGB& cloud);
	void get_region_colored_cloud(sensor_msgs::PointCloud2 &cloud_msg);

protected:

	bool load_parameters(std::string node_ns="");
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

public:

	// acquisition
	double acquisition_time_;
	std::string frame_id_;
	std::string acquisition_topic_;

	// filter and normal estimation
	int acquired_clouds_counter_;
	int meanK_;
	int k_search_;
	double stdv_threshold_;

	// region growing
	int rg_min_cluster_size_;
	int rg_max_cluster_size_;
	int rg_neightbors_;
	double rg_smoothness_threshold_;
	double rg_curvature_threshold_;

	// fast triangulation
	double tr_search_radius_;
	double tr_mu_;
	int tr_max_nearest_neighbors_;
	double tr_max_surface_angle_;
	double tr_min_angle_;
	double tr_max_angle_;
	bool tr_normal_consistency_;

	// voxel downsampling
	double voxel_leafsize_;

	double marker_alpha_;
	bool ignore_largest_cluster_;

protected:

	// roscpp members
	ros::Subscriber point_cloud_subs_;

	// pcl members
	pcl::PointCloud<pcl::PointXYZ>::Ptr acquired_cloud_ptr_;
	CloudRGB::Ptr region_colored_cloud_ptr_;
	std::vector<Cloud::Ptr> surface_clouds_;
	visualization_msgs::MarkerArray meshes_;

};

}} /* namespace godel_surface_detection */

#endif /* SURFACE_DETECTION_H_ */
