#ifndef PATH_GODEL_PROCESS_UTILS_H
#define PATH_GODEL_PROCESS_UTILS_H

#include <trajectory_msgs/JointTrajectory.h>

namespace godel_process_execution
{

void appendTrajectory(trajectory_msgs::JointTrajectory& original,
                      const trajectory_msgs::JointTrajectory& next);
}

#endif
