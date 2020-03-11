#include <ros/console.h>
#include <QtConcurrent/QtConcurrentRun>
#include "godel_simple_gui/blending_widget.h"
#include "godel_simple_gui/states/planning_state.h"  // previous
#include "godel_simple_gui/states/surface_select_state.h"  // next if fail
#include "godel_simple_gui/states/select_plans_state.h" // next is success
#include "godel_simple_gui/states/select_all_surface_state.h"

const static std::string PROCESS_PATH_SERVICE = "process_path";

godel_simple_gui::PlanningState::PlanningState()
  : process_planning_action_client_(PROCESS_PLANNING_ACTION_SERVER_NAME, true) {}
void godel_simple_gui::PlanningState::onStart(BlendingWidget& gui)
{
  gui.setText("Planning...");
  gui.setButtonsEnabled(false);
  gui_ptr_ = &gui;
  QObject::connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));
  QtConcurrent::run(this, &PlanningState::makeRequest, gui.options().pathPlanningParams());
}

void godel_simple_gui::PlanningState::onExit(BlendingWidget& gui) { gui.setButtonsEnabled(true); }


// Handlers for the fixed buttons
void godel_simple_gui::PlanningState::onNext(BlendingWidget& gui) {}
void godel_simple_gui::PlanningState::onBack(BlendingWidget& gui) {}
void godel_simple_gui::PlanningState::onReset(BlendingWidget& gui) {}


// State Specific Functions
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

void godel_simple_gui::PlanningState::setFeedbackText(QString feedback)
{
  gui_ptr_->appendText("\n" + feedback.toStdString());
}


// Action Callbacks
void godel_simple_gui::PlanningState::processPlanningDoneCallback(
    const actionlib::SimpleClientGoalState& state,
    const godel_msgs::ProcessPlanningResultConstPtr& result)
{
  if(result->succeeded)
    Q_EMIT newStateAvailable(new SelectPlansState());
  else
    Q_EMIT newStateAvailable(new SelectAllSurfaceState());
}

void godel_simple_gui::PlanningState::processPlanningActiveCallback()
{
  ROS_INFO_STREAM("Goal is active");
}

void godel_simple_gui::PlanningState::processPlanningFeedbackCallback(
    const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback)
{
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}

