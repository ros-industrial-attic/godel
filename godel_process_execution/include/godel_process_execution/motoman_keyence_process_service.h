#ifndef PATH_EXECUTION_SERVICE_H
#define PATH_EXECUTION_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/KeyenceProcessExecution.h>

namespace godel_process_execution
{

class ProcessExecutionService
{
public:
  ProcessExecutionService(const std::string& name, const std::string& sim_name, 
                       const std::string& real_name, ros::NodeHandle& nh);

  /**
   * Currently forwards the godel_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  bool executionCallback(godel_msgs::KeyenceProcessExecution::Request& req,
                         godel_msgs::KeyenceProcessExecution::Response& res);
private:
  ros::ServiceServer server_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  std::string name_;
};

}

#endif // PATH_EXECUTION_SERVICE_H
