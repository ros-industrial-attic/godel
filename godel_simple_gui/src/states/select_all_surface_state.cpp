#include <ros/console.h>
#include "godel_msgs/SelectSurface.h"
#include "godel_simple_gui/blending_widget.h"
#include "godel_simple_gui/states/select_all_surface_state.h"
#include "godel_simple_gui/states/scan_teach_state.h"
#include "godel_simple_gui/states/planning_state.h"

void godel_simple_gui::SelectAllSurfaceState::onStart(BlendingWidget& gui)
{
  const std::string SELECT_SURFACE_SERVICE = "select_surface";
  ros::ServiceClient client = gui.nodeHandle().serviceClient<godel_msgs::SelectSurface>(
      SELECT_SURFACE_SERVICE);
  godel_msgs::SelectSurface msg;
  msg.request.action = msg.request.SELECT_ALL;
  client.call(msg.request, msg.response);
  gui.setText("Press Next to select all surfaces.\nPress Back to select individual surfaces.");
}

void godel_simple_gui::SelectAllSurfaceState::onExit(BlendingWidget& gui) {}

void godel_simple_gui::SelectAllSurfaceState::onNext(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new PlanningState());
}

void godel_simple_gui::SelectAllSurfaceState::onBack(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new ScanTeachState());
}

void godel_simple_gui::SelectAllSurfaceState::onReset(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new ScanTeachState());
}
