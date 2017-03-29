#ifndef EXECUTING_STATE_H
#define EXECUTING_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

namespace godel_simple_gui
{

class ExecutingState : public GuiState
{
  Q_OBJECT
public:
  // Constructor
  ExecutingState(const std::vector<std::string>& plans);

  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

protected:
  void executeOne(const std::string &plan, BlendingWidget& gui);
  void executeAll(BlendingWidget& gui);

private:
  std::vector<std::string> plan_names_;
  ros::ServiceClient real_client_;
};
}

#endif // EXECUTING_STATE_H
