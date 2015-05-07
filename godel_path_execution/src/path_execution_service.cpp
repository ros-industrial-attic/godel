#include <godel_path_execution/path_execution_service.h>

#include <simulator_service/SimulateTrajectory.h>
#include <moveit_msgs/ExecuteKnownTrajectory.h>

godel_path_execution::PathExecutionService::PathExecutionService(const std::string& name, 
                                                                 const std::string& sim_name,
                                                                 const std::string& real_name,
                                                                 ros::NodeHandle& nh)
  : name_(name)
{
  // Communicates with the 
  real_client_ = nh.serviceClient<moveit_msgs::ExecuteKnownTrajectory>(real_name);

  sim_client_ = nh.serviceClient<simulator_service::SimulateTrajectory>(sim_name);

  server_ = nh.advertiseService<PathExecutionService,
                                godel_msgs::TrajectoryExecution::Request,
                                godel_msgs::TrajectoryExecution::Response>
            (name, &godel_path_execution::PathExecutionService::executionCallback, this);
}

bool godel_path_execution::PathExecutionService::executionCallback(godel_msgs::TrajectoryExecution::Request& req,
                                                                   godel_msgs::TrajectoryExecution::Response& res)
{
  using moveit_msgs::ExecuteKnownTrajectory;
  using simulator_service::SimulateTrajectory;

  ROS_INFO_STREAM("Godel Path Execution '" << name_ << "' recieved new trajectory.");

  if (req.simulate)
  {
    // Pass the trajectory to the simulation service
    SimulateTrajectory srv;
    srv.request.wait_for_execution = req.wait_for_execution;
    srv.request.trajectory = req.trajectory;

    if (sim_client_.call(srv))
    {
      // currently no response fields in the simulate header
      return true;
    }
  }
  else
  {
    // Pass the trajectory to moveit
    ExecuteKnownTrajectory srv;
    srv.request.wait_for_execution = req.wait_for_execution;
    srv.request.trajectory.joint_trajectory = req.trajectory;

    if (real_client_.call(srv))
    {
      res.code = srv.response.error_code.val;
      return true;
    }
  }

  return false;
}
