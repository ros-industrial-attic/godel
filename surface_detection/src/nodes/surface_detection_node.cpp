/*
 * surface_detection_node.cpp
 *
 *  Created on: Feb 12, 2014
 *      Author: ros-industrial
 */
#include <ros/ros.h>
#include <surface_detection/surface_detection.h>
#include <pcl/console/parse.h>

float const DEFAULT_ACQUISITION_TIME = 2.0f; //second
const std::string NODE_NAME = "surface_detection_node";
const std::string HELP_TEXT="\n" + NODE_NAME + " help:\n" +
		"-h help menu\n" +
		"-a <acquisition time (sec)>\n";

int main(int argc,char** argv)
{
	ros::init(argc,argv,"surface_detection_node");
	ros::NodeHandle nh;

	// parsing arguments
	float acquisition_time = DEFAULT_ACQUISITION_TIME;

	if(pcl::console::find_switch(argc,argv,"-h"))
	{
		std::cout<<HELP_TEXT<<std::endl;
		return 0;
	}

	if(pcl::console::find_switch(argc,argv,"-a"))
	{
		pcl::console::parse(argc,argv,"-a",acquisition_time);
	}

	ROS_INFO_STREAM("Using acquisition time '"<<acquisition_time<<"'");

	// surface detection instance
	surface_detection::SurfaceDetection sf;

	// acquire data
	sf.set_acquisition_time(acquisition_time);
	sf.acquire_data();

	sf.~SurfaceDetection();

}




