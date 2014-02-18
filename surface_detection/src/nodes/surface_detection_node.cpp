/*
 * surface_detection_node.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: ros-industrial
 */
#include <ros/ros.h>
#include <surface_detection/detection/surface_detection.h>
#include <surface_detection/interactive/interactive_surface_server.h>
#include <pcl/console/parse.h>

const float DEFAULT_ACQUISITION_TIME = 2.0f; //second
const std::string DEFAULT_FRAME_ID = "world_frame";

const std::string SEGMENTS_CLOUD_TOPIC = "segments_cloud";
const std::string MARKER_ARRAY_TOPIC = "segment_markers";
const std::string NODE_NAME = "surface_detection_node";
const std::string HELP_TEXT="\n" + NODE_NAME + " help:\n" +
		"-h help menu\n" +
		"-a <acquisition time (sec)>\n" +
		"-i <frame id>\n";

int main(int argc,char** argv)
{
	ros::init(argc,argv,"surface_detection_node");
	ros::NodeHandle nh;

	// parsing arguments
	float acquisition_time = DEFAULT_ACQUISITION_TIME;
	std::string frame_id = DEFAULT_FRAME_ID;

	if(pcl::console::find_switch(argc,argv,"-h"))
	{
		std::cout<<HELP_TEXT<<std::endl;
		return 0;
	}

	if(pcl::console::find_switch(argc,argv,"-a"))
	{
		pcl::console::parse(argc,argv,"-a",acquisition_time);
	}

	if(pcl::console::find_switch(argc,argv,"-i"))
	{
		pcl::console::parse(argc,argv,"-i",frame_id);
	}

	ROS_INFO_STREAM("Using acquisition time '"<<acquisition_time<<"'");

	// publishers
	ros::Publisher point_cloud_publisher = nh.advertise<sensor_msgs::PointCloud2>(
			SEGMENTS_CLOUD_TOPIC,1);

	ros::Publisher markers_publisher = nh.advertise<visualization_msgs::MarkerArray>(
			MARKER_ARRAY_TOPIC,1);

	// surface detection instance
	surface_detection::detection::SurfaceDetection sf;

	// marker server instance
	surface_detection::interactive::InteractiveSurfaceServer servr;

	// load paramters
	if(!sf.init() || !servr.init())
	{
		return 0;
	}

	// start server
	servr.run();

	// acquire data
	sf.set_acquisition_time(acquisition_time);
	bool succeeded = true;
	if(sf.acquire_data())
	{
		if(sf.find_surfaces())
		{
			ROS_INFO_STREAM(sf.get_results_summary());

		}
		else
		{
			ROS_ERROR_STREAM("No surface was found");
			succeeded = false;
		}
	}
	else
	{
		ROS_ERROR_STREAM("Data acquisition failed");
		succeeded = false;
	}

	if(succeeded)
	{
		ROS_INFO_STREAM("Publishing segments visuals");
		sensor_msgs::PointCloud2 cloud_msg;
		visualization_msgs::MarkerArray markers_msg = sf.get_surface_markers();
		sf.get_region_colored_cloud(cloud_msg);
		cloud_msg.header.frame_id = cloud_msg.header.frame_id.empty() ? frame_id : cloud_msg.header.frame_id;

		// adding markers to server
		for(int i =0;i < markers_msg.markers.size();i++)
		{
			servr.add_surface(markers_msg.markers[i]);
		}

		ros::Duration loop_rate(1.0f);
		while(succeeded && ros::ok() )
		{
			point_cloud_publisher.publish(cloud_msg);
			markers_publisher.publish(markers_msg);
			ros::spinOnce();

			loop_rate.sleep();
		}

		point_cloud_publisher.shutdown();
		markers_publisher.shutdown();
	}

	return 0;

}




