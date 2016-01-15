#ifndef SCAN_TOOL_CONFIGURATION_WINDOW_H
#define SCAN_TOOL_CONFIGURATION_WINDOW_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <ui_scan_tool_configuration_window.h>
#include <godel_msgs/ScanPlanParameters.h>

namespace godel_plugins
{

class ScanPlanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  ScanPlanConfigWidget(const godel_msgs::ScanPlanParameters& params);
  godel_msgs::ScanPlanParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();
  virtual void update_internal_values();
  virtual int get_quality_combobox_index();
  virtual int get_scan_method_enum_value();

  godel_msgs::ScanPlanParameters params_;
  Ui::ScanToolConfigurationWindow ui_;
};
}

#endif // SCAN_TOOL_CONFIGURATION_WINDOW_H
