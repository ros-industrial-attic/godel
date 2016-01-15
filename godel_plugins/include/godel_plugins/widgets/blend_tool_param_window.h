#ifndef BLEND_TOOL_PARAM_WINDOW_H
#define BLEND_TOOL_PARAM_WINDOW_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <ui_blend_tool_param_window.h>
#include <godel_msgs/BlendingPlanParameters.h>

namespace godel_plugins
{

class BlendingPlanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  BlendingPlanConfigWidget(const godel_msgs::BlendingPlanParameters& params);

  godel_msgs::BlendingPlanParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();

  virtual void update_internal_values();

  godel_msgs::BlendingPlanParameters params_;
  Ui::BlendToolParamWindow ui_;
};
}

#endif // BLEND_TOOL_PARAM_WINDOW_H
