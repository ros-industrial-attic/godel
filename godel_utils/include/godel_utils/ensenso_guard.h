#ifndef ENSENSO_GUARD_H
#define ENSESNO_GUARD_H

#include <ros/node_handle.h>
#include <ros/service_client.h>

namespace ensenso
{
enum Command {STOP=0, START=1};

class EnsensoGuard
{
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
public:
  EnsensoGuard();
  ~EnsensoGuard();
  void issueCommand(Command command);
};

}

#endif // ENSENSO_GUARD_H
