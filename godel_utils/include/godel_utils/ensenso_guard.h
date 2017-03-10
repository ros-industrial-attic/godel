#ifndef ENSENSO_GUARD_H
#define ENSESNO_GUARD_H

#include <ros/node_handle.h>
#include <ros/service_client.h>

namespace ensenso
{
class EnsensoGuard
{
private:
  ros::NodeHandle nh_;
  ros::ServiceClient client_;
public:
  EnsensoGuard();
  ~EnsensoGuard();
};

}

#endif // ENSENSO_GUARD_H
