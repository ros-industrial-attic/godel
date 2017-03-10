#include <godel_msgs/EnsensoCommand.h>
#include <godel_utils/ensenso_guard.h>

namespace ensenso
{
const static std::string SERVICE_NAME = "ensenso_manager";

EnsensoGuard::EnsensoGuard() : client_(nh_.serviceClient<godel_msgs::EnsensoCommand>(SERVICE_NAME))
{
  godel_msgs::EnsensoCommand msg;
  msg.request.action = godel_msgs::EnsensoCommandRequest::STOP;
  try
  {
    client_.call(msg);
  }
  catch (const std::exception e)
  {
    ROS_ERROR_STREAM("Exception while attempting to stop ensenso: " << e.what());
  }
}

EnsensoGuard::~EnsensoGuard()
{
  godel_msgs::EnsensoCommand msg;
  msg.request.action = godel_msgs::EnsensoCommandRequest::START;
  try
  {
    client_.call(msg);
  }
  catch (const std::exception e)
  {
    ROS_ERROR_STREAM("Exception while attempting to restart ensenso: " << e.what());
  }
}

}
