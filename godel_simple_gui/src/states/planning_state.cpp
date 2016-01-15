#include "godel_simple_gui/states/planning_state.h"
// previous
#include "godel_simple_gui/states/surface_select_state.h"
// next
#include "godel_simple_gui/states/wait_to_simulate_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

#include "godel_msgs/ProcessPlanning.h"

#include <QtConcurrentRun>

const static std::string PROCESS_PATH_SERVICE = "process_path";

void godel_simple_gui::PlanningState::onStart(BlendingWidget& gui)
{
  gui.setText("Planning...");
  gui.setButtonsEnabled(false);

  planning_client_ =
      gui.nodeHandle().serviceClient<godel_msgs::ProcessPlanning>(PROCESS_PATH_SERVICE);
  QtConcurrent::run(this, &PlanningState::makeRequest, gui.options().blendingParams(),
                    gui.options().scanParams());
}

void godel_simple_gui::PlanningState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void godel_simple_gui::PlanningState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::onReset(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::makeRequest(godel_msgs::BlendingPlanParameters blend_params,
                                                  godel_msgs::ScanPlanParameters scan_params)
{
  godel_msgs::ProcessPlanning srv;
  srv.request.action = srv.request.GENERATE_MOTION_PLAN_AND_PREVIEW;
  srv.request.params = blend_params;
  srv.request.scan_params = scan_params;
  srv.request.use_default_parameters = false;

  if (planning_client_.call(srv))
  {
    Q_EMIT newStateAvailable(new WaitToSimulateState());
  }
  else
  {
    ROS_WARN_STREAM("Process planning failed");
    Q_EMIT newStateAvailable(new SurfaceSelectState());
  }
}
