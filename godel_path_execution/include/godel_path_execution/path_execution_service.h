#ifndef PATH_EXECUTION_SERVICE_H
#define PATH_EXECUTION_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/TrajectoryExecution.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

namespace godel_path_execution
{

class PathExecutionService
{
public:
  PathExecutionService(ros::NodeHandle& nh);

  /**
   * Currently forwards the godel_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  bool executionCallback(godel_msgs::TrajectoryExecution::Request& req,
                         godel_msgs::TrajectoryExecution::Response& res);

private:
  ros::ServiceServer server_;
  actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac_;
  std::string name_;
};
}

#endif // PATH_EXECUTION_SERVICE_H
