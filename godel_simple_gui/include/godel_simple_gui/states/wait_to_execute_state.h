#ifndef WAIT_TO_EXECUTE_STATE_H
#define WAIT_TO_EXECUTE_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

namespace godel_simple_gui
{

class WaitToExecuteState : public GuiState
{
  Q_OBJECT
public:
  WaitToExecuteState(const std::vector<std::string>& plans);

  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

private:
  std::vector<std::string> plan_names_;
};
}
#endif // WAIT_FOR_EXECUTE_STATE_H
