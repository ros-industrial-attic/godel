/*
	Copyright Feb 17, 2014 Southwest Research Institute

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

#ifndef INTERACTIVE_SURFACE_SERVER_H_
#define INTERACTIVE_SURFACE_SERVER_H_

#include <ros/ros.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>

namespace interactive_markers
{
	typedef boost::shared_ptr<interactive_markers::InteractiveMarkerServer> InteractiveMarkerServerPtr;
}

namespace godel_surface_detection {
namespace interactive {

namespace params{
	static const std::string PARAMETER_NS = "interactive_surface_server";
}

namespace defaults{
	static const std::string MARKER_SERVER_NAME = "surface_marker_server";
	static const std::string FRAME_ID  = "world_frame";
	static const std::string MARKER_DESCRIPTION = "surface";
	static const std::string INTERACTIVE_MARKER_NAME = "select surface";
	static const int MAX_TRIANGLES = 20;
	static const double ARROW_LENGTH = 0.08f;
	static const double ARROW_SHAFT_DIAMETER = 0.02f;
	static const double ARROW_HEAD_DIAMETER = 0.04f;
	static const double ARROW_HEAD_LENGTH = 0.02f;
	static const double ARROW_DISTANCE = 0.02f;
}


class InteractiveSurfaceServer {

public:

	typedef std::pair<std::string, bool> Selection;
public:
	InteractiveSurfaceServer();
	virtual ~InteractiveSurfaceServer();


	bool init();
	void run();

	void add_surface(const visualization_msgs::Marker& marker);
	void add_surface(const visualization_msgs::Marker& marker,const geometry_msgs::Pose& pose);
	void add_random_surface_marker();
	void remove_all_surfaces();
	int get_surface_count()
	{
		return surface_selection_map_.size();
	}

protected:

	interactive_markers::InteractiveMarkerServerPtr marker_server_ptr_;
	interactive_markers::MenuHandler menu_handler_;
	std::map<std::string,bool> surface_selection_map_;

protected:


	bool load_parameters();
	void button_marker_callback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void menu_marker_callback(
			const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);
	void set_selection_flag(std::string marker_name,bool selected);
	void toggle_selection_flag(std::string marker_name);
	void create_polygon_marker(visualization_msgs::Marker& marker,int triangles);
	void create_arrow_marker(const visualization_msgs::Marker& surface_marker,
			visualization_msgs::Marker& arrow_marker);


protected:

	std::string marker_name_;
	std::string marker_description_;

	double arrow_length_;
	double arrow_shaft_diameter_;
	double arrow_head_diameter_;
	double arrow_head_length_;
	double arrow_distance_;

	// menu handling
	uint32_t select_entry_id_;
	uint32_t unselect_entry_id_;
	uint32_t select_all_entry_id_;
	uint32_t clear_all_entry_id_;
	uint32_t hide_entry_id_;
	uint32_t show_all_entry_id_;



	// callbacks
	interactive_markers::InteractiveMarkerServer::FeedbackCallback button_callback_;
	interactive_markers::InteractiveMarkerServer::FeedbackCallback menu_callback_;
};

} /* namespace interactive */
} /* namespace godel_surface_detection */

#endif /* INTERACTIVE_SURFACE_SERVER_H_ */
