#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "godel_msgs/SelectMotionPlan.h"
#include "godel_simple_gui/states/executing_state.h"
#include "godel_simple_gui/states/wait_to_execute_state.h"
#include "godel_simple_gui/states/error_state.h"
#include "godel_simple_gui/blending_widget.h"
#include "godel_simple_gui/states/scan_teach_state.h"

const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";

struct BadExecutionError
{
  BadExecutionError(const std::string& what) : what(what) {}
  std::string what;
};

godel_simple_gui::ExecutingState::ExecutingState(const std::vector<std::string>& plans)
    : plan_names_(plans)
{
}

void godel_simple_gui::ExecutingState::onStart(BlendingWidget& gui)
{
  gui.setText("Executing...");
  gui.setButtonsEnabled(false);

  real_client_ =
      gui.nodeHandle().serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
  QtConcurrent::run(this, &ExecutingState::executeAll, boost::ref(gui));
}

void godel_simple_gui::ExecutingState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }

void godel_simple_gui::ExecutingState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::ExecutingState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::ExecutingState::onReset(BlendingWidget& gui) {}

void godel_simple_gui::ExecutingState::executeAll(BlendingWidget& gui)
{
  try
  {
    for (std::size_t i = 0; i < plan_names_.size(); ++i)
    {
      executeOne(plan_names_[i], gui);
    }

    Q_EMIT newStateAvailable(new ScanTeachState());
  }
  catch (const BadExecutionError& err)
  {
    ROS_ERROR_STREAM("There was an error executing a plan " << err.what);
    Q_EMIT newStateAvailable(new ErrorState(err.what, new WaitToExecuteState(plan_names_)));
  }
}

void godel_simple_gui::ExecutingState::executeOne(const std::string& plan, BlendingWidget& gui)
{
  godel_msgs::SelectMotionPlanActionGoal goal;
  goal.goal.name = plan;
  goal.goal.simulate = false;
  goal.goal.wait_for_execution = true;
  gui.sendGoalAndWait(goal);
}


