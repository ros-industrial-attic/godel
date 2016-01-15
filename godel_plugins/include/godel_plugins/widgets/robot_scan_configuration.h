#ifndef ROBOT_SCAN_CONFIGURATION_H
#define ROBOT_SCAN_CONFIGURATION_H

#include <godel_plugins/widgets/parameter_window_base.h>
#include <godel_msgs/RobotScanParameters.h>

#include <tf/transform_datatypes.h>
#include <geometry_msgs/Pose.h>

#include <ui_robot_scan_configuration.h>
#include <ui_pose_widget.h>

namespace godel_plugins
{
/**
 * @brief The PoseWidget class
 */
class PoseWidget : public QWidget
{
  Q_OBJECT
public:
  PoseWidget(QWidget* parent = NULL);

  void set_values(const geometry_msgs::Pose& p);
  void set_values(const tf::Transform& t);
  tf::Transform get_values();

protected:
  Ui::PoseWidget ui_;
};

/**
 * @brief The RobotScanConfigWidget class
 */
class RobotScanConfigWidget : public ParameterWindowBase
{
  Q_OBJECT
public:
  RobotScanConfigWidget(const godel_msgs::RobotScanParameters& params);

  godel_msgs::RobotScanParameters& params() { return params_; }

protected:
  virtual void update_gui_fields();

  virtual void update_internal_values();

  godel_msgs::RobotScanParameters params_;
  PoseWidget* world_to_obj_pose_widget_;
  PoseWidget* tcp_to_cam_pose_widget_;
  Ui::RobotScanConfigWindow ui_;
};
}

#endif // ROBOT_SCAN_CONFIGURATION_H
