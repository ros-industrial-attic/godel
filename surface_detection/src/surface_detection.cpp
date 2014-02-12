/*
 * surface_detection.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: ros-industrial
 */

#include <surface_detection/surface_detection.h>

namespace surface_detection {

SurfaceDetection::SurfaceDetection() {
	// TODO Auto-generated constructor stub

}

SurfaceDetection::~SurfaceDetection() {
	// TODO Auto-generated destructor stub
}

void SurfaceDetection::acquire_data(float duration)
{
	// initialize  aggregate point cloud
	aggregate_cloud_ptr_.reset(new Cloud());
	acquired_clouds_counter_ = 0;

	// initialize subscriber
	ros::NodeHandle nh;
	point_cloud_subs_ = nh.subscribe(POINT_COLUD_TOPIC,1,
			&SurfaceDetection::point_cloud_subscriber_cb,this);

	ROS_INFO_STREAM("Started point cloud acquisition");

	ros::Duration acquistion_time(duration);
	ros::Time start_time = ros::Time::now();
	while(ros::ok() &&
			((start_time + acquistion_time) > ros::Time::now()))
	{
		ros::spinOnce();
	}

	point_cloud_subs_.shutdown();
	ROS_INFO_STREAM("Finished point cloud acquisition");
}

bool SurfaceDetection::find_surfaces()
{

}

void SurfaceDetection::point_cloud_subscriber_cb(const sensor_msgs::PointCloud2ConstPtr &msg)
{
	// convert to message to point cloud
	CloudPtr new_cloud_ptr = CloudPtr(new Cloud());
	pcl::fromROSMsg<pcl::PointXYZ>(*msg,*new_cloud_ptr);

	// add to aggregate point cloud
	(*aggregate_cloud_ptr_)+=*new_cloud_ptr;
	acquired_clouds_counter_++;

	ROS_INFO_STREAM("Added " <<acquired_clouds_counter_ <<" point clouds");
}

bool SurfaceDetection::apply_statistical_filter(const Cloud& in,Cloud& out)
{

}

bool SurfaceDetection::apply_region_growing_segmentation(const Cloud& in,
		std::vector<Cloud>& segments)
{

}

bool SurfaceDetection::apply_fast_triangulation(const std::vector<Cloud>& segments,
		std::vector<pcl::PolygonMesh>& meshes)
{

}


} /* namespace surface_detection */
