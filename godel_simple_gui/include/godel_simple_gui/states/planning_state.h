#ifndef PLANNING_STATE_H
#define PLANNING_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

#include "godel_msgs/BlendingPlanParameters.h"
#include "godel_msgs/ScanPlanParameters.h"

namespace godel_simple_gui
{

class PlanningState : public GuiState
{
  Q_OBJECT
public:
  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

private:
  void makeRequest(godel_msgs::BlendingPlanParameters blend_params,
                   godel_msgs::ScanPlanParameters scan_params);

  ros::ServiceClient planning_client_;
};
}

#endif // PLANNING_STATE_H
