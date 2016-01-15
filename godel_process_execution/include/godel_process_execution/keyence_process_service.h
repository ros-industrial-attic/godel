#ifndef KEYENCE_PROCESS_SERVICE_H
#define KEYENCE_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/KeyenceProcessExecution.h>

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
  bool executionCallback(godel_msgs::KeyenceProcessExecution::Request& req,
                         godel_msgs::KeyenceProcessExecution::Response& res);

  bool executeProcess(godel_msgs::KeyenceProcessExecution::Request& req);
  bool simulateProcess(godel_msgs::KeyenceProcessExecution::Request& req);

private:
  ros::ServiceServer server_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  ros::ServiceClient keyence_client_;
};
}

#endif // KEYENCE_PROCESS_SERVICE_H
