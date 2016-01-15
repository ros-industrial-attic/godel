#ifndef ABB_BLEND_PROCESS_SERVICE_H
#define ABB_BLEND_PROCESS_SERVICE_H

#include <ros/ros.h>
#include <godel_msgs/BlendProcessExecution.h>

namespace godel_process_execution
{

class AbbBlendProcessService
{
public:
  AbbBlendProcessService(ros::NodeHandle& nh);

  /**
   * Currently forwards the godel_msgs::TrajectoryExecution on to the corresponding
   * MoveIt node. The idea though is that abstracting 'execution' will give us more flexibility
   * later to implement our own process parameters related to execution.
   */
  bool executionCallback(godel_msgs::BlendProcessExecution::Request& req,
                         godel_msgs::BlendProcessExecution::Response& res);

  bool executeProcess(godel_msgs::BlendProcessExecution::Request req);

  bool simulateProcess(godel_msgs::BlendProcessExecution::Request req);

private:
  ros::ServiceServer server_;
  ros::ServiceClient real_client_;
  ros::ServiceClient sim_client_;
  bool j23_coupled_;
};
}

#endif // ABB_BLEND_PROCESS_SERVICE_H
