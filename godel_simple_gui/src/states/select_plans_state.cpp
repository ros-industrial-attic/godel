#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "godel_simple_gui/states/select_plans_state.h"
// previous state
#include "godel_simple_gui/states/surface_select_state.h"
// next state
#include "godel_simple_gui/states/simulating_state.h"
// reset state
#include "godel_simple_gui/states/scan_teach_state.h"
// error
#include "godel_simple_gui/states/error_state.h"
#include "godel_simple_gui/blending_widget.h"
#include "godel_msgs/GetAvailableMotionPlans.h"
#include "godel_simple_gui/states/select_all_surface_state.h"

const static std::string GET_AVAILABLE_MOTION_PLANS_SERVICE = "get_available_motion_plans";

void godel_simple_gui::SelectPlansState::onStart(BlendingWidget& gui)
{
  ros::ServiceClient client = gui.nodeHandle().serviceClient<godel_msgs::GetAvailableMotionPlans>(
      GET_AVAILABLE_MOTION_PLANS_SERVICE);

  godel_msgs::GetAvailableMotionPlans srv;
  if (client.call(srv))
  {
    this->plan_names_ = srv.response.names;
  }
  else
  {
    ROS_WARN_STREAM("Could not fetch plan names");
  }

  if (plan_names_.empty())
  {
    Q_EMIT newStateAvailable(new SelectAllSurfaceState());
  }
  else
  {
    gui.showPlanListWidget();
    gui.setLabelText("Select Plan:");
    gui.addPlans(plan_names_);
  }

}

void godel_simple_gui::SelectPlansState::onExit(BlendingWidget& gui) {}

// Handlers for the fixed buttons
void godel_simple_gui::SelectPlansState::onNext(BlendingWidget& gui)
{
  gui.setLabelText("System Status:");
  gui.showStatusWindow();
  if (gui.planSelectionEmpty())
    Q_EMIT newStateAvailable(new SimulatingState(plan_names_));
  else
    Q_EMIT newStateAvailable(new SimulatingState(gui.getPlanNames()));
}

void godel_simple_gui::SelectPlansState::onBack(BlendingWidget& gui)
{
  gui.setLabelText("System Status:");
  gui.showStatusWindow();
  Q_EMIT newStateAvailable(new SurfaceSelectState());
}

void godel_simple_gui::SelectPlansState::onReset(BlendingWidget& gui)
{
  gui.setLabelText("System Status:");
  Q_EMIT newStateAvailable(new ScanTeachState());
}
