#include <godel_msgs/EnsensoCommand.h>
#include <godel_utils/ensenso_guard.h>

namespace ensenso
{
const static std::string SERVICE_NAME = "ensenso_manager";

EnsensoGuard::EnsensoGuard() : client_(nh_.serviceClient<godel_msgs::EnsensoCommand>(SERVICE_NAME))
{
  issueCommand(Command::STOP);
}

EnsensoGuard::~EnsensoGuard()
{
  issueCommand(Command::START);
}

void EnsensoGuard::issueCommand(Command command)
{
  godel_msgs::EnsensoCommand msg;
    msg.request.action = command;
    try
    {
      client_.call(msg);
    }
    catch (const std::exception& e)
    {
      std::string command_type;
      switch(command)
      {
        case Command::STOP :
        {
          command_type = "stop";
        }
        case Command::START :
        {
          command_type = "start";
        }
      }

      ROS_ERROR_STREAM("Exception while attempting to " << command_type <<  " ensenso: " << e.what());
    }
}

}
