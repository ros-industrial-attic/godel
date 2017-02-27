#include "godel_simple_gui/states/planning_state.h"
// previous
#include "godel_simple_gui/states/surface_select_state.h"
// next
#include "godel_simple_gui/states/wait_to_simulate_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

#include <QtConcurrent/QtConcurrentRun>

const static std::string PROCESS_PATH_SERVICE = "process_path";

godel_simple_gui::PlanningState::PlanningState()
  : process_planning_action_client_(PROCESS_PLANNING_ACTION_SERVER_NAME, true) {}

void godel_simple_gui::PlanningState::onStart(BlendingWidget& gui)
{
  gui.setText("Planning...");
  gui.setButtonsEnabled(false);
  QtConcurrent::run(this, &PlanningState::makeRequest, gui.options().pathPlanningParams());
}

void godel_simple_gui::PlanningState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }

// Handlers for the fixed buttons
void godel_simple_gui::PlanningState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::onReset(BlendingWidget& gui) {}

void godel_simple_gui::PlanningState::makeRequest(godel_msgs::PathPlanningParameters planning_params)
{
  godel_msgs::ProcessPlanningGoal goal;
  goal.action = godel_msgs::ProcessPlanningGoal::GENERATE_MOTION_PLAN_AND_PREVIEW;
  goal.params = planning_params;
  process_planning_action_client_.sendGoal(
        goal,
        boost::bind(&godel_simple_gui::PlanningState::processPlanningDoneCallback, this, _1, _2),
        boost::bind(&godel_simple_gui::PlanningState::processPlanningActiveCallback, this),
        boost::bind(&godel_simple_gui::PlanningState::processPlanningFeedbackCallback, this, _1));
  ROS_INFO_STREAM("Goal sent from planning state");
}

void godel_simple_gui::PlanningState::processPlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const godel_msgs::ProcessPlanningResultConstPtr& result)
{
  if(result->succeeded)
  {
    Q_EMIT newStateAvailable(new WaitToSimulateState());
  }
  else
  {
    ROS_WARN_STREAM("Process planning failed");
    Q_EMIT newStateAvailable(new SurfaceSelectState());
  }
}


void godel_simple_gui::PlanningState::processPlanningActiveCallback()
{
  // Not implemented
  ROS_INFO_STREAM("Goal is active");
}

void godel_simple_gui::PlanningState::processPlanningFeedbackCallback(
    const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback)
{
  // ToDo: Tie feedback to visual element
  ROS_INFO("Got Feedback: %s", (feedback->last_completed).c_str());
}
