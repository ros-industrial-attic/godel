#include <godel_surface_detection/services/surface_blending_service.h>

#include <godel_msgs/TrajectoryExecution.h>

// Process Execution
#include <godel_msgs/BlendProcessExecution.h>
#include <godel_msgs/KeyenceProcessExecution.h>
// Process Planning
#include <godel_msgs/BlendProcessPlanning.h>
#include <godel_msgs/KeyenceProcessPlanning.h>
// Param server
#include <godel_param_helpers/godel_param_helpers.h>
#include <fstream>
#include <iostream>

const static std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const static std::string SELECT_SURFACE_SERVICE = "select_surface";
const static std::string PROCESS_PATH_SERVICE = "process_path";
const std::string GET_AVAILABLE_MOTION_PLANS_SERVICE = "get_available_motion_plans";
const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";
const static std::string SURFACE_BLENDING_PARAMETERS = "surface_blending_parameters";

int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_auto_blending");
  ros::NodeHandle nh;
  ros::Rate loop_rate(10);

  std::string ns = ros::this_node::getNamespace();
  while (ros::ok())
  {
	//Get parameters for surface detection
    godel_msgs::SurfaceBlendingParameters param_srv;
    param_srv.request.action = param_srv.request.GET_CURRENT_PARAMETERS;
    ros::ServiceClient param_client = nh.serviceClient<godel_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS);
    std::vector<std::string> plan_names;

    if (param_client.call(param_srv))
    {
	  //Scan and detect surface
	  godel_msgs::SurfaceDetection surfacedetection_srv;
	  surfacedetection_srv.request.action = 3;
	  surfacedetection_srv.request.use_default_parameters = false;
	  surfacedetection_srv.request.robot_scan = param_srv.response.robot_scan;
	  surfacedetection_srv.request.surface_detection = param_srv.response.surface_detection;
	  ros::ServiceClient surface_client_ = nh.serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);

	  if (!surface_client_.call(surfacedetection_srv))
	  {
	    ROS_WARN_STREAM("Unable to call surface detection service");
	  }

	  //Select all surface
	  godel_msgs::SelectSurface::Request req;
	  godel_msgs::SelectSurface::Response res;
	  req.action = req.SELECT_ALL;
	  ros::ServiceClient select_surface_client_ = nh.serviceClient<godel_msgs::SelectSurface>(SELECT_SURFACE_SERVICE);
	  bool succeeded = select_surface_client_.call(req, res);

	  //Generate path
	  godel_msgs::ProcessPlanning process_plan;
	  process_plan.request.use_default_parameters = false;
	  process_plan.request.params = param_srv.response.blending_plan;
	  process_plan.request.scan_params =param_srv.response.scan_plan;
	  process_plan.request.action = 2;
	  ros::ServiceClient process_plan_client_ = nh.serviceClient<godel_msgs::ProcessPlanning>(PROCESS_PATH_SERVICE);

	  if (process_plan_client_.call(process_plan))
	  {
	    godel_msgs::GetAvailableMotionPlans motionQuery_srv;
	    ros::ServiceClient get_motion_plans_client_ = nh.serviceClient<godel_msgs::GetAvailableMotionPlans>(GET_AVAILABLE_MOTION_PLANS_SERVICE);
	    if (get_motion_plans_client_.call(motionQuery_srv))
	    {
	      plan_names = motionQuery_srv.response.names;
	    }
	    else
	    {
	      ROS_ERROR_STREAM("Could not get names from 'available motions server'");
	    }
	  }

	  //Blending Simulation
	  for (std::size_t i = 0; i < plan_names.size(); ++i)
	  {
	    godel_msgs::SelectMotionPlan motion_srv;
	    motion_srv.request.name = plan_names[i];
	    motion_srv.request.simulate = true;
	    motion_srv.request.wait_for_execution = true;
	    ros::ServiceClient sim_client_ =  nh.serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
	    if (!sim_client_.call(motion_srv))
	    {
	      ROS_WARN_STREAM("Client simulation request returned false");
	    }
	  }
	}
	ros::spinOnce();
   	loop_rate.sleep();
  }
}
