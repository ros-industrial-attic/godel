#ifndef GODEL_SURFACE_DETECTION_TIMING_UTILITY_H
#define GODEL_SURFACE_DETECTION_TIMING_UTILITY_H

#include <ros/time.h>
#include <ros/console.h>

namespace godel_surface_detection
{

class TimingUtility
{
public:
  TimingUtility(const std::string& name, int index = -1)
    : name_(name)
    , index_(index)
    , start_(ros::Time::now())
  {}

  ros::Duration timeSoFar() const
  {
    return ros::Time::now() - start_;
  }

  void info() const
  {
    if (index_ > -1)
      ROS_INFO("%s - %d: duration = %f seconds", name_.c_str(), index_, timeSoFar().toSec());
    else
      ROS_INFO("%s: duration = %f seconds", name_.c_str(), timeSoFar().toSec());
  }

  ~TimingUtility()
  {
    info();
  }

private:
  const std::string name_;
  const int index_;
  ros::Time start_;
};

}

#endif
