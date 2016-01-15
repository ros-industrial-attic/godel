#ifndef BLEND_PROCESS_SERVICE_H
#define BLEND_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/BlendProcessExecution.h>

namespace godel_process_execution
{

class BlendProcessService
{
public:
  BlendProcessService(ros::NodeHandle& nh);

  bool executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                         godel_msgs::BlendProcessExecution::Response& res);

  bool executeProcess(godel_msgs::BlendProcessExecution::Request req);

  bool simulateProcess(godel_msgs::BlendProcessExecution::Request req);

private:
  ros::ServiceServer server_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
};
}

#endif // BLEND_PROCESS_SERVICE_H
