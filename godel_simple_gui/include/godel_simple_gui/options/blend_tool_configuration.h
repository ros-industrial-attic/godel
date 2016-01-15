#ifndef BLEND_TOOL_CONFIGURATION_H
#define BLEND_TOOL_CONFIGURATION_H

#include "godel_simple_gui/parameter_window_base.h"

#include "godel_msgs/BlendingPlanParameters.h"

namespace Ui
{
class BlendToolConfigWindow;
}

namespace godel_simple_gui
{

class BlendingPlanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  BlendingPlanConfigWidget(godel_msgs::BlendingPlanParameters params);

  godel_msgs::BlendingPlanParameters& params() { return params_; }

  virtual void update_display_fields();

  virtual void update_internal_fields();

protected:
  godel_msgs::BlendingPlanParameters params_;
  Ui::BlendToolConfigWindow* ui_;
};
}

#endif // BLEND_TOOL_CONFIGURATION_H
