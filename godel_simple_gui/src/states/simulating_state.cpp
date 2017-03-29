#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "godel_msgs/SelectMotionPlan.h"
#include "godel_simple_gui/blending_widget.h"
#include "godel_simple_gui/states/simulating_state.h"
#include "godel_simple_gui/states/wait_to_execute_state.h"
#include <iostream>

const static std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";

godel_simple_gui::SimulatingState::SimulatingState(const std::vector<std::string>& plans)
    : plan_names_(plans)
{
}

void godel_simple_gui::SimulatingState::onStart(BlendingWidget& gui)
{
  gui.setText("Simulating...");
  gui.setButtonsEnabled(false);

  sim_client_ =
      gui.nodeHandle().serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
  QtConcurrent::run(this, &SimulatingState::simulateAll, boost::ref(gui));
}

void godel_simple_gui::SimulatingState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void godel_simple_gui::SimulatingState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::SimulatingState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::SimulatingState::onReset(BlendingWidget& gui) {}

void godel_simple_gui::SimulatingState::simulateAll(BlendingWidget& gui)
{
  for (std::size_t i = 0; i < plan_names_.size(); ++i)
  {
    simulateOne(plan_names_[i], gui);
  }

  Q_EMIT newStateAvailable(new WaitToExecuteState(plan_names_));
}

void godel_simple_gui::SimulatingState::simulateOne(const std::string& plan, BlendingWidget& gui)
{
  godel_msgs::SelectMotionPlanActionGoal goal;
  goal.goal.name = plan;
  goal.goal.simulate = true;
  goal.goal.wait_for_execution = true;
  gui.sendGoalAndWait(goal);
}
