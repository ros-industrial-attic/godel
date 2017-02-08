#ifndef PATH_PLANNING_PARAM_WINDOW_H
#define PATH_PLANNING_PARAM_WINDOW_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <ui_path_planning_param_window.h>
#include <godel_msgs/PathPlanningParameters.h>

namespace godel_plugins
{

class PathPlanningConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  PathPlanningConfigWidget(const godel_msgs::PathPlanningParameters& params);

  godel_msgs::PathPlanningParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();

  virtual void update_internal_values();

  godel_msgs::PathPlanningParameters params_;
  Ui::PathPlanningParamWindow ui_;
};
}

#endif // BLEND_TOOL_PARAM_WINDOW_H
