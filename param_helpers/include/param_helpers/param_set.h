#ifndef PARAM_SET_H
#define PARAM_SET_H

#include <param_helpers/param_interfaces.h>
#include <param_helpers/yaml_interface.h>
#include <param_helpers/rosparam_interface.h>

#include <fstream>

namespace param_helpers
{
  template<typename Data>
  inline void readFromParamServer(Data& data, const std::string& ns = "")
  {
    RosParamReaderWriter param_server(ns);
    ParamInterfaces<Data>::load(param_server, data);
  }

  template<typename Data>
  inline void saveToYamlFile(const Data& data, const std::string& file, const std::string& ns = "")
  {
    YamlFileWriter yaml_file (file, ns);
    ParamInterfaces<Data>::save(yaml_file, data);
  }

  template<typename Data>
  inline void readFromYamlFile(Data& data, const std::string& file, const std::string& ns = "")
  {
    YamlFileReader yaml_file (file, ns);
    ParamInterfaces<Data>::load(yaml_file, data);
  }

  inline bool checkForYamlFile(const std::string& file)
  {
    std::ifstream f (file.c_str());
    if (f.good()) return true;
    else return false;
  }

  template<typename Data>
  inline void attemptCacheLoad(Data& data, const std::string& file_name, const std::string& ns = "")
  {
    ROS_INFO_STREAM("Attempting to load parameters from file  " << file_name << " with ns " << ns);
    if (checkForYamlFile(file_name))
    {
      readFromYamlFile(data, file_name, ns);
    }
    else
    {
      ROS_INFO_STREAM("Defaulting to parameter server");
      readFromParamServer(data, "~/" + ns);
    }
  }

}

#endif
