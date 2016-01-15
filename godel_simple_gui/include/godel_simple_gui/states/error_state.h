#ifndef ERROR_STATE_H
#define ERROR_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

namespace godel_simple_gui
{

class ErrorState : public GuiState
{
  Q_OBJECT
public:
  ErrorState(const std::string& msg, GuiState* next_state);

  // Entry and exit classes
  virtual void onStart(BlendingWidget& gui);
  virtual void onExit(BlendingWidget& gui);

  // Handlers for the fixed buttons
  virtual void onNext(BlendingWidget& gui);
  virtual void onBack(BlendingWidget& gui);
  virtual void onReset(BlendingWidget& gui);

private:
  std::string msg_;
  GuiState* next_state_;
  QWidget* window_;
};
}

#endif // ERROR_STATE_H
