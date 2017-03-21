#ifndef BLEND_PROCESS_SERVICE_H
#define BLEND_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/ProcessExecutionAction.h>
#include <actionlib/server/simple_action_server.h>

namespace godel_process_execution
{

class BlendProcessService
{
public:
  BlendProcessService(ros::NodeHandle& nh);

  void executionCallback(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool executeProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool simulateProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  actionlib::SimpleActionServer<godel_msgs::ProcessExecutionAction> process_exe_action_server_;
  bool j23_coupled_;
};
}

#endif // BLEND_PROCESS_SERVICE_H
