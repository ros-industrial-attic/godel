#include "godel_simple_gui/options_submenu.h"

#include "ui_options_select_menu.h"

#include "godel_simple_gui/options/robot_scan_configuration.h"

godel_simple_gui::OptionsSubmenu::OptionsSubmenu(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::OptionsSubmenu();
  ui_->setupUi(this);
  // Set up option menus
  //// Robot Scan
  robot_scan_ = new RobotScanConfigWidget(godel_msgs::RobotScanParameters());
  connect(ui_->pushButtonScanOptions, SIGNAL(clicked()), robot_scan_, SLOT(show()));
  //// Surface Detection
  surface_detection_ = new SurfaceDetectionConfigWidget(godel_msgs::SurfaceDetectionParameters());
  connect(ui_->pushButtonSurfaceOptions, SIGNAL(clicked()), surface_detection_, SLOT(show()));
  //// Path Planning
  path_planning_params_ = new PathPlanningConfigWidget(godel_msgs::PathPlanningParameters());
  connect(ui_->pushButtonPathPlanningOptions, SIGNAL(clicked()), path_planning_params_, SLOT(show()));
  //// Scan (QA) params
  scan_params_ = new ScanPlanConfigWidget(godel_msgs::ScanPlanParameters());
  connect(ui_->pushButtonQAOptions, SIGNAL(clicked()), scan_params_, SLOT(show()));

  connect(robot_scan_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(surface_detection_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(path_planning_params_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
  connect(scan_params_, SIGNAL(parameters_save_requested()), this, SIGNAL(saveRequested()));
}

const godel_msgs::RobotScanParameters& godel_simple_gui::OptionsSubmenu::robotScanParams() const
{
  return robot_scan_->params();
}

void godel_simple_gui::OptionsSubmenu::setRobotScanParams(
    const godel_msgs::RobotScanParameters& params)
{
  robot_scan_->params() = params;
  robot_scan_->update_display_fields();
}

const godel_msgs::SurfaceDetectionParameters&
godel_simple_gui::OptionsSubmenu::surfaceDetectionParams() const
{
  return surface_detection_->params();
}

void godel_simple_gui::OptionsSubmenu::setSurfaceDetectionParams(
    const godel_msgs::SurfaceDetectionParameters& params)
{
  surface_detection_->params() = params;
  surface_detection_->update_display_fields();
}

const godel_msgs::PathPlanningParameters& godel_simple_gui::OptionsSubmenu::pathPlanningParams() const
{
  return path_planning_params_->params();
}

void godel_simple_gui::OptionsSubmenu::setPathPlanningParams(const godel_msgs::PathPlanningParameters& params)
{
  path_planning_params_->params() = params;
  path_planning_params_->update_display_fields();
}

const godel_msgs::ScanPlanParameters& godel_simple_gui::OptionsSubmenu::scanParams() const
{
  return scan_params_->params();
}

void godel_simple_gui::OptionsSubmenu::setScanParams(const godel_msgs::ScanPlanParameters& params)
{
  scan_params_->params() = params;
  scan_params_->update_display_fields();
}
