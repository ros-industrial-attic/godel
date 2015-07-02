#include <godel_process_execution/motoman_blend_process_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

godel_process_execution::ProcessExecutionService::ProcessExecutionService(const std::string& name, 
                                                                          const std::string& sim_name,
                                                                          const std::string& real_name,
                                                                          ros::NodeHandle& nh)
  : name_(name)
{
  ROS_INFO_STREAM("Starting process execution service with name " << name << ". Using simulation service '" 
    << sim_name << "' and actual execution service: '" << real_name << "'.");

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<ProcessExecutionService,
                                godel_msgs::BlendProcessExecution::Request,
                                godel_msgs::BlendProcessExecution::Response>
            (name, &godel_process_execution::ProcessExecutionService::executionCallback, this);
}

bool godel_process_execution::ProcessExecutionService::executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                                                                         godel_msgs::BlendProcessExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;

  if (req.simulate)
  {
    // Pass the trajectory to the simulation service
    SimulateTrajectory srv_approach;
    srv_approach.request.wait_for_execution = req.wait_for_execution;
    srv_approach.request.trajectory = req.trajectory_approach;

    // Pass the trajectory to the simulation service
    SimulateTrajectory srv_process;
    srv_process.request.wait_for_execution = req.wait_for_execution;
    srv_process.request.trajectory = req.trajectory_process;

    // Pass the trajectory to the simulation service
    SimulateTrajectory srv_depart;
    srv_depart.request.wait_for_execution = req.wait_for_execution;
    srv_depart.request.trajectory = req.trajectory_depart;

    if (!sim_client_.call(srv_approach))
    {
      // currently no response fields in the simulate header
      return false;
    }
    if (!sim_client_.call(srv_process))
    {
      // currently no response fields in the simulate header
      return false;
    }
    if (!sim_client_.call(srv_depart))
    {
      // currently no response fields in the simulate header
      return false;
    }
    return true;
    ROS_WARN_STREAM("PathExecutionService: Failed to call simulation service");
  }
  else
  {
    ExecuteKnownTrajectory srv_approach;
    srv_approach.request.wait_for_execution = req.wait_for_execution;
    srv_approach.request.trajectory.joint_trajectory = req.trajectory_approach;

    ExecuteKnownTrajectory srv_process;
    srv_process.request.wait_for_execution = req.wait_for_execution;
    srv_process.request.trajectory.joint_trajectory = req.trajectory_process;

    ExecuteKnownTrajectory srv_depart;
    srv_depart.request.wait_for_execution = req.wait_for_execution;
    srv_depart.request.trajectory.joint_trajectory = req.trajectory_depart;

    if (!real_client_.call(srv_approach))
    {
      res.code = srv_approach.response.error_code.val;
      return false;
    }
    if (!real_client_.call(srv_process))
    {
      res.code = srv_process.response.error_code.val;
      return false;
    }
    if (!real_client_.call(srv_depart))
    {
      res.code = srv_depart.response.error_code.val;
      return false;
    }
    return true;
    ROS_WARN_STREAM("PathExecutionService: Failed to call hardware execution service");
  }


  return false;
}
