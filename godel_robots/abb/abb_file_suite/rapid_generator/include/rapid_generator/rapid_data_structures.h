#ifndef RAPID_DATA_STRUCTURES_H
#define RAPID_DATA_STRUCTURES_H

#include <vector>
#include <string>

namespace rapid_emitter
{

/**
 * @brief A joint trajectory point that can be converted to a RAPID
 *        joint target.
 */
struct TrajectoryPt
{
  typedef std::vector<double> value_type;

  TrajectoryPt(const std::vector<double>& positions, double duration = 0.0)
      : positions_(positions), duration_(duration)
  {
  }

  value_type positions_; // degrees
  double duration_;      // seconds
};

/**
 * @brief This structure is used in combination with a vector of trajectory points to
 *        generate any entire rapid program. Some fields aren't used by the default
 *        software, but interact with proprietary extensions like Wolf Robotic's
 *        Wolfware.
 */
struct ProcessParams
{
  double spindle_speed;    // Tool rotation speed
  double tcp_speed;        // Tool linear traversal speed
  double force;            // For non-wolfware software, a general identification of force
  std::string output_name; // The I/O that toggles the tool power; We assume 0 is off and 1 is on.
  bool wolf_mode;          // In the case of Wolfware software, we emit special instructions.
  double slide_force;      // For WolfWare, a meaure of the cross-slide force (?)
};
}

#endif // RAPID_DATA_STRUCTURES_H
