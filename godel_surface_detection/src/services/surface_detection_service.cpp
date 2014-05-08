/*
	Copyright May 7, 2014 Southwest Research Institute

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

#include <godel_surface_detection/scan/robot_scan.h>
#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <godel_msgs/SurfaceDetection.h>
#include <godel_msgs/SelectSurface.h>
#include <godel_msgs/SelectedSurfacesChanged.h>
#include <pcl/console/parse.h>


const std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const std::string SELECT_SURFACE_SERVICE = "select_surface";
const std::string SELECTED_SURFACES_CHANGED_TOPIC = "selected_surfaces_changed";
const std::string ROBOT_SCAN_PATH_PREVIEW_TOPIC = "robot_scan_path_preview";

class SurfaceDetectionService
{
public:
	SurfaceDetectionService()
	{

	}

	~SurfaceDetectionService()
	{

	}

	bool init()
	{
		using namespace godel_surface_detection;

		// initializing surface detector
		if(surface_detection_.load_parameters("~/surface_detection") && robot_scan_.load_parameters("~/robot_scan") &&
				surface_server_.load_parameters())
		{
			// save default parameters
			default_robot_scan_params__ = robot_scan_.params_;
			default_surf_detection_params_ = surface_detection_.params_;


			ROS_INFO_STREAM("Surface detection service loaded parameters successfully");
			if(surface_detection_.init() && robot_scan_.init() && surface_server_.init())
			{
				// adding callbacks
				scan::RobotScan::ScanCallback cb = boost::bind(&detection::SurfaceDetection::add_cloud,&surface_detection_,_1);
				robot_scan_.add_scan_callback(cb);
				ROS_INFO_STREAM("Surface detection service initialization succeeded");
			}
			else
			{
				ROS_ERROR_STREAM("Surface detection service had an initialization error");
			}

		}
		else
		{
			ROS_ERROR_STREAM("Surface detection service failed to load parameters");
		}

		// start server
		interactive::InteractiveSurfaceServer::SelectionCallback f =	boost::bind(
				&SurfaceDetectionService::publish_selected_surfaces_changed,this);
		surface_server_.add_selection_callback(f);

		// initializing ros interface
		ros::NodeHandle nh;
		surface_detect_server_ = nh.advertiseService(SURFACE_DETECTION_SERVICE,
				&SurfaceDetectionService::surface_detection_server_callback,this);

		select_surface_server_ = nh.advertiseService(SELECT_SURFACE_SERVICE,
				&SurfaceDetectionService::select_surface_server_callback,this);

		selected_surf_changed_pub_ = nh.advertise<godel_msgs::SelectedSurfacesChanged>(SELECTED_SURFACES_CHANGED_TOPIC,1);

		return true;
	}

	void run()
	{
		surface_server_.run();
	}

protected:

	void publish_selected_surfaces_changed()
	{
		godel_msgs::SelectedSurfacesChanged msg;
		msg.selected_surfaces.clear();
		surface_server_.get_selected_list(msg.selected_surfaces);
		selected_surf_changed_pub_.publish(msg);
	}

	bool run_robot_scan(visualization_msgs::MarkerArray &surfaces)
	{
		bool succeeded = true;

		// publishing scan path preview
		robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);

		// clear all results
		surface_detection_.clear_results();

		// saving parameters used

		ROS_INFO_STREAM("Starting scan");

		int scans_completed = robot_scan_.scan(false);
		if(scans_completed > 0)
		{
			ROS_INFO_STREAM("Scan points reached "<<scans_completed);
			if(surface_detection_.find_surfaces())
			{
				// clear current surfaces
				surface_server_.remove_all_surfaces();

				// adding markers to server
				visualization_msgs::MarkerArray markers_msg = surface_detection_.get_surface_markers();
				for(int i =0;i < markers_msg.markers.size();i++)
				{
					surface_server_.add_surface(markers_msg.markers[i]);
				}

				// copying to output argument
				surfaces.markers.insert(surfaces.markers.begin(),markers_msg.markers.begin(),markers_msg.markers.end());

				// saving latest successful results
				latest_results_.robot_scan = robot_scan_.params_;
				latest_results_.surface_detection = surface_detection_.params_;
				latest_results_.surfaces_found = true;
				latest_results_.surfaces = surfaces;
				robot_scan_.get_latest_scan_poses(latest_results_.robot_scan_poses);
			}
			else
			{
				succeeded = false;
			}
		}
		else
		{
			succeeded = false;
			ROS_ERROR_STREAM("Scan failed");
		}
		return succeeded;
	}

	bool surface_detection_server_callback(godel_msgs::SurfaceDetection::Request &req,
			godel_msgs::SurfaceDetection::Response &res)
	{

		res.surfaces_found = false;
		res.surfaces = visualization_msgs::MarkerArray();

		switch(req.action)
		{
		case req.GET_CURRENT_PARAMETERS:
			res.robot_scan = robot_scan_.params_;
			res.surface_detection = surface_detection_.params_;
			break;

		case req.GET_DEFAULT_PARAMETERS:
			res.robot_scan = default_robot_scan_params__;
			res.surface_detection = default_surf_detection_params_;
			break;

		case req.PUBLISH_SCAN_PATH:

			if(req.use_default_parameters)
			{
				robot_scan_.params_ = default_robot_scan_params__;
				//surface_detection_.params_ = default_surf_detection_params_;
			}
			else
			{
				robot_scan_.params_ = req.robot_scan;
				//surface_detection_.params_ = req.surface_detection;
			}

			robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);
			break;

		case req.SCAN_AND_FIND_ONLY:

			if(req.use_default_parameters)
			{
				robot_scan_.params_ = default_robot_scan_params__;
				surface_detection_.params_ = default_surf_detection_params_;
			}
			else
			{
				robot_scan_.params_ = req.robot_scan;
				surface_detection_.params_ = req.surface_detection;
			}

			res.surfaces_found =  run_robot_scan(res.surfaces);
			res.surfaces.markers.clear();
			break;

		case req.SCAN_FIND_AND_RETURN:

			if(req.use_default_parameters)
			{
				robot_scan_.params_ = default_robot_scan_params__;
				surface_detection_.params_ = default_surf_detection_params_;
			}
			else
			{
				robot_scan_.params_ = req.robot_scan;
				surface_detection_.params_ = req.surface_detection;
			}

			res.surfaces_found =  run_robot_scan(res.surfaces);
			break;

		case req.GET_LATEST_RESULTS:

			res = latest_results_;
			break;

		}

		return true;
	}

	bool select_surface_server_callback(godel_msgs::SelectSurface::Request &req, godel_msgs::SelectSurface::Response &res)
	{
		switch(req.action)
		{
		case req.SELECT:

			for(int i = 0; req.select_surfaces.size();i++)
			{
				surface_server_.set_selection_flag(req.select_surfaces[i],true);
			}
			break;

		case req.DESELECT:

			for(int i = 0; req.select_surfaces.size();i++)
			{
				surface_server_.set_selection_flag(req.select_surfaces[i],false);
			}
			break;

		case req.SELECT_ALL:

			surface_server_.select_all(true);
			break;

		case req.DESELECT_ALL:

			surface_server_.select_all(false);
			break;

		case req.HIDE_ALL:

			surface_server_.show_all(false);
			break;

		case req.SHOW_ALL:
			surface_server_.show_all(true);
			break;
		}

		return true;
	}



protected:

	ros::ServiceServer surface_detect_server_;
	ros::ServiceServer select_surface_server_;
	ros::Publisher selected_surf_changed_pub_;

	// robot scan instance
	godel_surface_detection::scan::RobotScan robot_scan_;

	// surface detection instance
	godel_surface_detection::detection::SurfaceDetection surface_detection_;

	// marker server instance
	godel_surface_detection::interactive::InteractiveSurfaceServer surface_server_;

	// default parameters
	godel_msgs::RobotScanParameters default_robot_scan_params__;
	godel_msgs::SurfaceDetectionParameters default_surf_detection_params_;
	godel_msgs::SurfaceDetection::Response latest_results_;

};

int main(int argc,char** argv)
{
	ros::init(argc,argv,"surface_detection_server");
	ros::AsyncSpinner spinner(4);
	spinner.start();
	SurfaceDetectionService service;
	if(service.init())
	{
		service.run();
	}

	ros::waitForShutdown();
}

