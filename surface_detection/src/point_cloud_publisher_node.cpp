/*
 * point_cloud_publisher.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: ros-industrial
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

using namespace boost::filesystem;

const std::string POINT_CLOUD_TOPIC="sensor_point_cloud";
const std::string DATA_DIR = "data";
const float NOISE_THRESHOLD = 0.005f;

int main(int argc,char** argv)
{
	ros::init(argc,argv,"point_cloud_publisher_node");

	// point cloud publisher
	ros::NodeHandle nh;
	ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
			POINT_CLOUD_TOPIC,1);

	// pcl data
	path file_path;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());
	pcl::PointCloud<pcl::PointXYZ>::Ptr noise_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>());

	if(argc > 1)
	{
		file_path = path(ros::package::getPath("surface_detection") + "/"+
				+ DATA_DIR.c_str() + "/"+ std::string(argv[1]));

		if(exists(file_path))
		{
			ROS_INFO_STREAM("Found file: "<<file_path.c_str());
		}
		else
		{
			ROS_ERROR_STREAM("Must supply valid pcd file name that exists in 'data' directory");
		}

	}
	else
	{
		ROS_ERROR_STREAM("Must supply valid pcd file name that exists in 'data' directory");
		return 0;
	}


	// reading pcd file
	if(pcl::io::loadPCDFile(std::string(file_path.c_str()),*cloud_ptr) == -1)
	{
		ROS_ERROR_STREAM("Failed to read pcd file");
		return 0;
	}
	else
	{
		ROS_ERROR_STREAM("Successfully read pcd file with "
				<<cloud_ptr->points.size()<<" points");
	}

	// publishing
	ros::Duration loop_time(0.2f);
	srand(time(NULL));
	float noise;
	while(ros::ok())
	{

		// adding noise
		pcl::copyPointCloud(*cloud_ptr,*noise_cloud_ptr);
		for(int i = 0; i < noise_cloud_ptr->points.size();i++)
		{
			noise = float(rand())/float(RAND_MAX); // <0,1>
			noise = -NOISE_THRESHOLD*0.5f + noise*NOISE_THRESHOLD;
			pcl::PointXYZ &p = noise_cloud_ptr->points[i];
			noise_cloud_ptr->points[i].x = p.x + noise;
			noise_cloud_ptr->points[i].y = p.y + noise;
			noise_cloud_ptr->points[i].z = p.z + noise;
		}

		// convert to msg
		sensor_msgs::PointCloud2 msg;
		pcl::toROSMsg(*noise_cloud_ptr,msg);

		point_cloud_pub.publish(msg);
		loop_time.sleep();
	}

	return 0;
}



