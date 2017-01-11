#include "services/trajectory_library.h"

#include <rosbag/bag.h>
#include <rosbag/view.h>

void godel_surface_detection::TrajectoryLibrary::save(const std::string& filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Write);
  ros::Time now = ros::Time::now();

  for (TrajectoryMap::const_iterator it = map_.begin(); it != map_.end(); ++it)
  {
    bag.write(it->first, now, it->second);
  }
}

void godel_surface_detection::TrajectoryLibrary::load(const std::string& filename)
{
  rosbag::Bag bag;
  bag.open(filename, rosbag::bagmode::Read);
  rosbag::View view(bag);

  for (rosbag::View::iterator it = view.begin(); it != view.end(); ++it)
  {
    godel_msgs::ProcessPlanPtr ptr = it->instantiate<godel_msgs::ProcessPlan>();
    if (!ptr)
    {
      // message failed to load
      throw std::runtime_error("Could not load joint trajectory from ROS bag");
    }

    // Check to see if key is already in data structure
    std::string const& key = it->getTopic();
    if (map_.find(key) != map_.end())
    {
      throw std::runtime_error("Bagfile had name matching key already in data structure");
    }

    map_[key] = *ptr;
  }
}
