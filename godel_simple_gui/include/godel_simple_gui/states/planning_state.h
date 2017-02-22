#ifndef PLANNING_STATE_H
#define PLANNING_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <godel_msgs/PathPlanningParameters.h>
#include <godel_msgs/ProcessPlanningAction.h>

const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME = "process_planning_as";

namespace godel_simple_gui
{

class PlanningState : public GuiState
{
public:
  PlanningState() : process_planning_action_client_(PROCESS_PLANNING_ACTION_SERVER_NAME, true) {}
  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

private:
  Q_OBJECT
  void makeRequest(godel_msgs::PathPlanningParameters);
  ros::NodeHandle nh_;
  actionlib::SimpleActionClient<godel_msgs::ProcessPlanningAction> process_planning_action_client_;
  void processPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
              const godel_msgs::ProcessPlanningResultConstPtr& result);
  void processPlanningActiveCallback();
  void processPlanningFeedbackCallback(const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback);
};
}

#endif // PLANNING_STATE_H
