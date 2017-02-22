#ifndef PATH_PLANNING_CONFIGURATION_H
#define PATH_PLANNING_CONFIGURATION_H

#include "godel_simple_gui/parameter_window_base.h"
#include "godel_msgs/PathPlanningParameters.h"

namespace Ui
{
class PathPlanningConfigWindow;
}

namespace godel_simple_gui
{

class PathPlanningConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  PathPlanningConfigWidget(godel_msgs::PathPlanningParameters params);

  godel_msgs::PathPlanningParameters& params() { return params_; }

  virtual void update_display_fields();

  virtual void update_internal_fields();

protected:
  godel_msgs::PathPlanningParameters params_;
  Ui::PathPlanningConfigWindow* ui_;
};
}

#endif // PATH_PLANNING_CONFIGURATION_H
