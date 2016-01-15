#ifndef ROBOT_SCAN_CONFIGURATION_H
#define ROBOT_SCAN_CONFIGURATION_H

#include "godel_simple_gui/parameter_window_base.h"
#include "godel_simple_gui/options/pose_widget.h"

#include "godel_msgs/RobotScanParameters.h"

namespace Ui
{
class RobotScanConfigWindow;
}

namespace godel_simple_gui
{

class RobotScanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  RobotScanConfigWidget(godel_msgs::RobotScanParameters params);

  virtual ~RobotScanConfigWidget();

  godel_msgs::RobotScanParameters& params() { return params_; }
  const godel_msgs::RobotScanParameters& params() const { return params_; }

  virtual void update_display_fields();
  virtual void update_internal_fields();

protected:
  godel_msgs::RobotScanParameters params_;
  PoseWidget* world_to_obj_pose_widget_;
  PoseWidget* tcp_to_cam_pose_widget_;
  Ui::RobotScanConfigWindow* ui_;
};
}
#endif
