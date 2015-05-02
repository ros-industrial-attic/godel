#ifndef ROSPARAM_INTERFACE_H
#define ROSPARAM_INTERFACE_H

#include <ros/ros.h>

namespace param_helpers
{

class RosParamReaderWriter
{
public:
  RosParamReaderWriter(const std::string& ns)
    : nh_(ns)
  {}

  template<class T>
  void set(const std::string& key, const T& value)
  {
    nh_.setParam(key, value);
  }

  template<class T>
  void get(const std::string& key, T& value)
  {
    if (!nh_.getParam(key, value))
    {
      std::ostringstream os;
      os << "Could not load param " << key << " from server";
      throw std::runtime_error(os.str());
    }
      
  }

private:
  ros::NodeHandle nh_;
};

}

#endif