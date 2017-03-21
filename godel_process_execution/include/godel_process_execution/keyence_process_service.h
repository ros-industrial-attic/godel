#ifndef KEYENCE_PROCESS_SERVICE_H
#define KEYENCE_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <godel_msgs/ProcessExecutionAction.h>

namespace godel_process_execution
{

class KeyenceProcessService
{
public:
  KeyenceProcessService(ros::NodeHandle& nh);

  /**
   * Currently forwards the godel_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  void executionCallback(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool executeProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);
  bool simulateProcess(const godel_msgs::ProcessExecutionGoalConstPtr &goal);

private:
  ros::NodeHandle nh_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  actionlib::SimpleActionServer<godel_msgs::ProcessExecutionAction> process_exe_action_server_;
  ros::ServiceClient keyence_client_;
  ros::ServiceClient reset_scan_server_;
};
}

#endif // KEYENCE_PROCESS_SERVICE_H
