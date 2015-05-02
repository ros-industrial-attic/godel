#ifndef BLENDING_PARAMS_HELPER_H
#define BLENDING_PARAMS_HELPER_H

#include <param_helpers/param_interfaces.h>
#include <godel_msgs/BlendingPlanParameters.h>

namespace param_helpers 
{

template<>
struct ParamHelpers<godel_msgs::BlendingPlanParameters>
{
  template<typename Setter>
  static void save(Setter& setter, const std::string& ns, const godel_msgs::BlendingPlanParameters& params)
  {
    setter.set("some_data", params.margin);
  }

  template<typename Getter>
  static void load(Getter& getter, const std::string& ns, godel_msgs::BlendingPlanParameters& params)
  {
    getter.get("some_data", params.margin);
  }
};

}

#endif