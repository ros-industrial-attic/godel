#ifndef TRAJECTORY_LIBRARY_H
#define TRAJECTORY_LIBRARY_H

#include <string>
#include <map>

#include <godel_msgs/ProcessPlan.h>

namespace godel_surface_detection
{

class TrajectoryLibrary
{
public:
  typedef std::map<std::string, godel_msgs::ProcessPlan> TrajectoryMap;

  void load(const std::string& filename);
  void save(const std::string& filename);

  TrajectoryMap& get() { return map_; }
  const TrajectoryMap& get() const { return map_; }

private:
  TrajectoryMap map_;
};
}

#endif
