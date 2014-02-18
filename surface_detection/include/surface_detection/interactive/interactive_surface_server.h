/*
 * interactive_surface_server.h
 *
 *  Created on: Feb 17, 2014
 *      Author: ros-industrial
 */

#ifndef INTERACTIVE_SURFACE_SERVER_H_
#define INTERACTIVE_SURFACE_SERVER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace interactive_markers
{
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
}

namespace surface_detection {
namespace interactive {

namespace params{
	static const std::string FRAME_ID = "frame_id";
}

namespace defaults{
	static const std::string MARKER_SERVER_NAME = "surface_marker_server";
	static const std::string FRAME_ID  = "world_frame";
	static const std::string MARKER_DESCRIPTION = "surface";
	static const std::string INTERACTIVE_MARKER_NAME = "select surface";
	static const int MAX_TRIANGLES = 20;
	static const double ARROW_LENGTH = 0.04f;
	static const double ARROW_SHAFT_DIAMETER = 0.01f;
	static const double ARROW_HEAD_DIAMETER = 0.02f;
	static const double ARROW_HEAD_LENGTH = 0.01;
}


class InteractiveSurfaceServer {
public:
	InteractiveSurfaceServer();
	virtual ~InteractiveSurfaceServer();


	bool init();
	void run();

	void add_marker(const visualization_msgs::Marker& marker);
	void add_marker(const visualization_msgs::Marker& marker,const geometry_msgs::Pose& pose);
	void add_random_surface_marker();

protected:

	interactive_markers::InteractiveMarkerServerPtr marker_server_ptr_;
	interactive_markers::MenuHandler menu_handler_;
	std::vector<std::string> marker_names_;

protected:


	bool load_parameters();
	void button_marker_callback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void menu_marker_callback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void create_polygon_marker(visualization_msgs::Marker& marker,int triangles);
	void create_arrow_marker(const visualization_msgs::Marker& surface_marker,
			visualization_msgs::Marker& arrow_marker);


protected:

	std::string frame_id_;
	std::string marker_name_;
	std::string marker_description_;

	double arrow_lenght_;
	double arrow_shaft_diameter_;
	double arrow_head_diameter_;
	double arrow_head_length_;


	// callbacks
	interactive_markers::InteractiveMarkerServer::FeedbackCallback button_callback_;
	interactive_markers::InteractiveMarkerServer::FeedbackCallback menu_callback_;
};

} /* namespace interactive */
} /* namespace surface_detection */

#endif /* INTERACTIVE_SURFACE_SERVER_H_ */
