#ifndef SCAN_TEACH_STATE_H
#define SCAN_TEACH_STATE_H

#include "godel_simple_gui/gui_state.h"
#include <ros/ros.h>

namespace godel_simple_gui
{

class ScanTeachState : public GuiState
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
};
}

#endif
