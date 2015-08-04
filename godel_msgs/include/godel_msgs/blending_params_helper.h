#ifndef BLENDING_PARAMS_HELPER_H
#define BLENDING_PARAMS_HELPER_H

#include <param_helpers/param_interfaces.h>
#include <godel_msgs/BlendingPlanParameters.h>

namespace param_helpers 
{

template<>
struct ParamInterfaces<godel_msgs::BlendingPlanParameters>
{
  template<typename Setter>
  static void save(Setter& setter, const godel_msgs::BlendingPlanParameters& params)
  {
    setter.set("tool_radius", params.tool_radius);
    setter.set("margin", params.margin);
    setter.set("overlap", params.overlap);
    setter.set("approach_spd", params.approach_spd);
    setter.set("blending_spd", params.blending_spd);
    setter.set("retract_spd", params.retract_spd);
    setter.set("traverse_spd", params.traverse_spd);
    setter.set("discretization", params.discretization);
    setter.set("safe_traverse_height", params.safe_traverse_height);
    setter.set("min_boundary_length", params.min_boundary_length);
    setter.set("tool_force", params.tool_force);
    setter.set("spindle_speed", params.spindle_speed);
  }

  template<typename Getter>
  static void load(Getter& getter, godel_msgs::BlendingPlanParameters& params)
  {
    getter.get("tool_radius", params.tool_radius);
    getter.get("margin", params.margin);
    getter.get("overlap", params.overlap);
    getter.get("approach_spd", params.approach_spd);
    getter.get("blending_spd", params.blending_spd);
    getter.get("retract_spd", params.retract_spd);
    getter.get("traverse_spd", params.traverse_spd);
    getter.get("discretization", params.discretization);
    getter.get("safe_traverse_height", params.safe_traverse_height);
    getter.get("min_boundary_length", params.min_boundary_length);
    getter.get("tool_force", params.tool_force);
    getter.get("spindle_speed", params.spindle_speed);
  }
};

}

#endif