#include "godel_simple_gui/states/scanning_state.h"
#include "godel_simple_gui/states/surface_select_state.h"
#include "godel_simple_gui/states/scan_teach_state.h"
#include "godel_simple_gui/blending_widget.h"

#include <ros/console.h>

#include <QtConcurrent/QtConcurrentRun>

// Scan params
#include "godel_msgs/SurfaceDetection.h"
#include "godel_msgs/RobotScanParameters.h"
#include "godel_msgs/SurfaceDetectionParameters.h"

// Class level constants
const static std::string SURFACE_DETECTION_SERVICE = "surface_detection";

void godel_simple_gui::ScanningState::onStart(BlendingWidget& gui)
{
  gui.setButtonsEnabled(false);
  gui.setText("Scanning...");
  surface_client_ =
      gui.nodeHandle().serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);
  QtConcurrent::run(this, &ScanningState::makeRequest, gui.options().robotScanParams(),
                    gui.options().surfaceDetectionParams());
}

void godel_simple_gui::ScanningState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void godel_simple_gui::ScanningState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::ScanningState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::ScanningState::onReset(BlendingWidget& gui) {}

void godel_simple_gui::ScanningState::makeRequest(
    godel_msgs::RobotScanParameters scan_params,
    godel_msgs::SurfaceDetectionParameters surface_params)
{
  godel_msgs::SurfaceDetection srv;
  srv.request.action = srv.request.SCAN_AND_FIND_ONLY;
  srv.request.use_default_parameters = false;
  srv.request.robot_scan = scan_params;
  srv.request.surface_detection = surface_params;
  if (!surface_client_.call(srv))
  {
    ROS_WARN_STREAM("Unable to call surface detection service");
    Q_EMIT newStateAvailable(new ScanTeachState());
  }
  else
  {
    if(srv.response.surfaces_found)
      Q_EMIT newStateAvailable(new SurfaceSelectState());
    else
      Q_EMIT newStateAvailable(new ScanTeachState());
  }
}
