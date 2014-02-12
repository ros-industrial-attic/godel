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

namespace surface_detection {

typedef pcl::PointCloud<pcl::PointXYZ> Cloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;

static const std::string POINT_COLUD_TOPIC="sensor_point_cloud";
class SurfaceDetection {


public:
	SurfaceDetection();
	virtual ~SurfaceDetection();

public:

	void acquire_data(float duration);
	bool find_surfaces();

protected:


	void point_cloud_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &msg);
	bool apply_statistical_filter(const Cloud& in,Cloud& out);
	bool apply_region_growing_segmentation(const Cloud& in,
			std::vector<Cloud>& segments);
	bool apply_fast_triangulation(const std::vector<Cloud>& segments,
			std::vector<pcl::PolygonMesh>& meshes);


protected:

	ros::Subscriber point_cloud_subs_;
	pcl::PointCloud<pcl::PointXYZ>::Ptr aggregate_cloud_ptr_;
	int acquired_clouds_counter_;
};

} /* namespace surface_detection */

#endif /* SURFACE_DETECTION_H_ */
