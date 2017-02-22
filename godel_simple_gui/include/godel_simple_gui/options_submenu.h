#ifndef OPTIONS_SUBMENU_H
#define OPTIONS_SUBMENU_H

#include <QWidget>

#include "godel_simple_gui/options/robot_scan_configuration.h"
#include "godel_simple_gui/options/surface_detection_configuration.h"
#include "godel_simple_gui/options/scan_tool_configuration.h"
#include "godel_simple_gui/options/path_planning_configuration.h"

namespace Ui
{
class OptionsSubmenu;
}

namespace godel_simple_gui
{

class OptionsSubmenu : public QWidget
{
  Q_OBJECT

public:
  OptionsSubmenu(QWidget* parent = 0);

  // For every submenu / set of parameters, we have getters/setters
  const godel_msgs::RobotScanParameters& robotScanParams() const;
  void setRobotScanParams(const godel_msgs::RobotScanParameters& params);

  const godel_msgs::SurfaceDetectionParameters& surfaceDetectionParams() const;
  void setSurfaceDetectionParams(const godel_msgs::SurfaceDetectionParameters& params);

  const godel_msgs::PathPlanningParameters& pathPlanningParams() const;
  void setPathPlanningParams(const godel_msgs::PathPlanningParameters& params);

  const godel_msgs::ScanPlanParameters& scanParams() const;
  void setScanParams(const godel_msgs::ScanPlanParameters& params);

Q_SIGNALS:
  void saveRequested();

private:
  // Display layout
  Ui::OptionsSubmenu* ui_;
  // Configuration components
  RobotScanConfigWidget* robot_scan_;
  SurfaceDetectionConfigWidget* surface_detection_;
  PathPlanningConfigWidget* path_planning_params_;
  ScanPlanConfigWidget* scan_params_;
};

} // end namespace godel_simple_gui

#endif // OPTIONS_SUBMENU_H
