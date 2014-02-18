/*
 * interactive_surface_server.cpp
 *
 *  Created on: Feb 17, 2014
 *      Author: ros-industrial
 */

#include <surface_detection/interactive/interactive_surface_server.h>
#include <tf/transform_datatypes.h>
#include <tf/tf.h>
#include <sstream>

namespace surface_detection {
namespace interactive {

InteractiveSurfaceServer::InteractiveSurfaceServer() :
		frame_id_(defaults::FRAME_ID),
		marker_name_(defaults::MARKER_SERVER_NAME),
		marker_description_(defaults::MARKER_DESCRIPTION)
{
	// TODO Auto-generated constructor stub

}

InteractiveSurfaceServer::~InteractiveSurfaceServer() {
	// TODO Auto-generated destructor stub
}


bool InteractiveSurfaceServer::init()
{

	srand(time(NULL));
	return load_parameters();
}

void InteractiveSurfaceServer::run()
{
	marker_server_ptr_ = interactive_markers::InteractiveMarkerServerPtr(
			new interactive_markers::InteractiveMarkerServer(defaults::MARKER_SERVER_NAME,"",false));

	// create callbacks
	button_callback_ =  interactive_markers::InteractiveMarkerServer::FeedbackCallback(
			boost::bind(&InteractiveSurfaceServer::button_marker_callback,this,_1));

	menu_callback_ = interactive_markers::InteractiveMarkerServer::FeedbackCallback(
				boost::bind(&InteractiveSurfaceServer::menu_marker_callback,this,_1));

	// setup menu handler
	menu_handler_.insert("Option 1",menu_callback_);
	menu_handler_.insert("Option 2",menu_callback_);
	interactive_markers::MenuHandler::EntryHandle submenu_handle = menu_handler_.insert("Submenu");
	menu_handler_.insert(submenu_handle,"Submenu Option 1",menu_callback_);
	menu_handler_.insert(submenu_handle,"Submenu Option 2",menu_callback_);

	marker_server_ptr_->applyChanges();
}

bool InteractiveSurfaceServer::load_parameters()
{
	ros::NodeHandle nh("~");
	bool succeeded = nh.getParam(params::FRAME_ID,frame_id_);
	if(succeeded)
	{
		ROS_INFO_STREAM("surface server finished loading parameters");
	}
	else
	{
		ROS_ERROR_STREAM("surface server failed to load parameters");
	}
	return succeeded;
}

void InteractiveSurfaceServer::button_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" menu control was clicked");
		marker_server_ptr_->applyChanges();
		break;
	}
}

void InteractiveSurfaceServer::menu_marker_callback(
		const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
	switch(feedback->event_type)
	{
	case visualization_msgs::InteractiveMarkerFeedback::BUTTON_CLICK:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" button control was clicked");
		marker_server_ptr_->applyChanges();
		break;

	case visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT:
		ROS_INFO_STREAM("marker "<<feedback->marker_name <<" menu control was clicked");
		marker_server_ptr_->applyChanges();
		break;
	}
}

void InteractiveSurfaceServer::create_arrow_marker(const visualization_msgs::Marker& surface_marker,
		visualization_msgs::Marker& arrow_marker)
{

}

void InteractiveSurfaceServer::add_marker(const visualization_msgs::Marker& marker,
		const geometry_msgs::Pose &pose)
{
	// create marker
	std::stringstream ss;
	ss<<marker_name_<<"_" <<marker_names_.size() +1;
	visualization_msgs::InteractiveMarker int_marker;
	int_marker.name = ss.str();
	int_marker.pose = pose;

	// create button control
	visualization_msgs::InteractiveMarkerControl button_control;
	button_control.interaction_mode = button_control.BUTTON;
	button_control.markers.push_back(marker);
	button_control.name = "button_" + int_marker.name;
	button_control.always_visible = true;

	// fill interactive marker
	int_marker.controls.push_back(button_control);
	int_marker.scale = 1;
	int_marker.header.frame_id = frame_id_;
	int_marker.description = marker_description_;

	// add marker to server
	marker_server_ptr_->insert(int_marker,button_callback_);
	menu_handler_.apply(*marker_server_ptr_,int_marker.name);

	// save name
	marker_names_.push_back(int_marker.name);

	// apply changes
	marker_server_ptr_->applyChanges();
}

void InteractiveSurfaceServer::add_marker(const visualization_msgs::Marker& marker)
{
	geometry_msgs::Pose pose;
	tf::poseTFToMsg(tf::Transform::getIdentity(),pose);
	add_marker(marker,pose);
}

void InteractiveSurfaceServer::add_random_surface_marker()
{
	// create pose
	geometry_msgs::Pose pose;
	double xy_step = 0.2f;
	double z_step = 0.1f;
	double pmin = -2;
	double pmax = 2;
	tfScalar x= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;
	tfScalar y= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;;
	tfScalar z= (pmax-pmin)*(static_cast<double>(rand())/static_cast<double>(RAND_MAX)) + pmin;;
	tf::Transform t = tf::Transform(tf::Quaternion::getIdentity(),tf::Vector3(x,y,z));
	tf::poseTFToMsg(t,pose);

	// create triangle list marker
	visualization_msgs::Marker marker;
	create_polygon_marker(marker,rand() % defaults::MAX_TRIANGLES + 1);

	add_marker(marker,pose);
}


void InteractiveSurfaceServer::create_polygon_marker(
		visualization_msgs::Marker& marker,int triangles)
{

	ROS_INFO_STREAM("Creating polygon of "<<triangles<<" triangles");

	// setting marker properties
	marker.type = marker.TRIANGLE_LIST;
	marker.scale.x = marker.scale.y = marker.scale.z = 1;
	marker.pose.position.x = marker.pose.position.y = marker.pose.position.z = 0;
	marker.pose.orientation.x = marker.pose.orientation.y = marker.pose.orientation.z =0;
	marker.pose.orientation.w = 1;
	marker.color.r = 0.5f;
	marker.color.g = 0.5f;
	marker.color.b = 0.8f;
	marker.color.a = 0.4f;

	// defining triangle vertices
	tf::Vector3 p0(-0.05f,0,0);
	tf::Vector3 p1(0.05f,0,0);
	tf::Vector3 p2;

	// rotation transform
	tf::Transform rot(tf::Quaternion(tf::Vector3(0,0,1),M_PI/3.0f),tf::Vector3(0,0,0));

	for(int i = 0; i < triangles;i++)
	{
		// computing last triangle point
		p2 = rot*(p1-p0) + p0;

		// creating temp vector
		std::vector<tf::Vector3> points;
		points.push_back(p0);points.push_back(p1);points.push_back(p2);

		// adding points to marker
		for(unsigned int j = 0 ; j< points.size();j++)
		{
			geometry_msgs::Point p;
			p.x = points[j].getX();p.y = points[j].getY();p.z = points[j].getZ();
			marker.points.push_back(p);
		}

		// assigning new points for next triangle
		if(rand()%2 == 0)
		{
			p1 = p2;
		}
		else
		{
			p0 = p2;
		}
	}
}



} /* namespace interactive */
} /* namespace surface_detection */
