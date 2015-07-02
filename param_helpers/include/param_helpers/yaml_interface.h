#ifndef YAML_INTERFACE_H
#define YAML_INTERFACE_H

#include <yaml-cpp/yaml.h>
#include <fstream>

namespace param_helpers
{

inline
std::vector<std::string> splitString(const std::string& s, char delim, const std::string& prefix = "")
{
  std::istringstream ss(s);
  std::string token;
  std::vector<std::string> tokens;
  if (!prefix.empty()) tokens.push_back(prefix);
  while (std::getline(ss, token, delim))
  {
    if (!token.empty()) tokens.push_back(token);
  }
  return tokens;
}

inline
YAML::Node getNode(YAML::Node n, const std::vector<std::string>& tokens)
{
  for (unsigned i = 0; i < tokens.size(); ++i) n.reset(n[tokens[i]]);
  return n;
}

class YamlFileReader
{
public:
  YamlFileReader(const std::string& filename, const std::string prefix = "")
    : filename_(filename)
    , prefix_(prefix)
    , node_(YAML::LoadFile(filename))
    {}

  template<class T>
  void get(const std::string& key, T& value)
  {
    std::vector<std::string> tokens = splitString(key, '/', prefix_);
    YAML::Node n = getNode(node_, tokens);
    value = n.as<T>();
  }

  private:
    std::string filename_, prefix_;
    YAML::Node node_;
};

class YamlFileWriter
{
public:
  YamlFileWriter(const std::string& filename, const std::string prefix = "")
    : filename_(filename)
    , prefix_(prefix)
    , node_(YAML::NodeType::Map)
    {
    }

  template<class T>
  void set(const std::string& key, const T& value)
  {
    std::vector<std::string> tokens = splitString(key, '/', prefix_);
    YAML::Node n = getNode(node_, tokens);
    n = value;
  }

   ~YamlFileWriter()
  {
    std::ofstream ofile (filename_.c_str());
    if (!ofile) ROS_ERROR_STREAM("Could not save parameters on exit");
    ofile << node_;
  }

private:
  std::string filename_, prefix_;
  YAML::Node node_;
};



}

#endif