#ifndef SCANNING_STATE_H
#define SCANNING_STATE_H

#include "godel_simple_gui/gui_state.h"

#include <ros/ros.h>

#include "godel_msgs/RobotScanParameters.h"
#include "godel_msgs/SurfaceDetectionParameters.h"

namespace godel_simple_gui
{
class ScanningState : public GuiState
{
  Q_OBJECT
public:
  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

private:
  void makeRequest(godel_msgs::RobotScanParameters scan_params,
                   godel_msgs::SurfaceDetectionParameters surface_params);

  ros::ServiceClient surface_client_;
};
}

#endif
