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
  //// Blending params
  blend_params_ = new BlendingPlanConfigWidget(godel_msgs::BlendingPlanParameters());
  connect(ui_->pushButtonBlendOptions, SIGNAL(clicked()), blend_params_, SLOT(show()));
  //// Scan (QA) params
  scan_params_ = new ScanPlanConfigWidget(godel_msgs::ScanPlanParameters());
  connect(ui_->pushButtonQAOptions, SIGNAL(clicked()), scan_params_, SLOT(show()));
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

const godel_msgs::BlendingPlanParameters& godel_simple_gui::OptionsSubmenu::blendingParams() const
{
  return blend_params_->params();
}

void godel_simple_gui::OptionsSubmenu::setBlendingParams(
    const godel_msgs::BlendingPlanParameters& params)
{
  blend_params_->params() = params;
  blend_params_->update_display_fields();
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
