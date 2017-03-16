#include "godel_simple_gui/states/wait_to_execute_state.h"
// previous state
#include "godel_simple_gui/states/select_plans_state.h"
// next state
#include "godel_simple_gui/states/executing_state.h"
// reset state
#include "godel_simple_gui/states/scan_teach_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

godel_simple_gui::WaitToExecuteState::WaitToExecuteState(const std::vector<std::string>& plans)
    : plan_names_(plans)
{
}

void godel_simple_gui::WaitToExecuteState::onStart(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToExecuteState start");
  gui.setText("Ready to Execute.\nPress 'Next' to begin.");
}

void godel_simple_gui::WaitToExecuteState::onExit(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToExecuteState exit");
}

// Handlers for the fixed buttons
void godel_simple_gui::WaitToExecuteState::onNext(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToExecuteState next");
  Q_EMIT newStateAvailable(new ExecutingState(plan_names_));
}

void godel_simple_gui::WaitToExecuteState::onBack(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToExecuteState back");
  Q_EMIT newStateAvailable(new SelectPlansState());
}

void godel_simple_gui::WaitToExecuteState::onReset(BlendingWidget& gui)
{
  ROS_INFO_STREAM("WaitToExecuteState reset");
  Q_EMIT newStateAvailable(new ScanTeachState());
}
