#ifndef SELECT_ALL_SURFACE_STATE_H
#define SELECT_ALL_SURFACE_STATE_H

#include "godel_simple_gui/gui_state.h"

namespace godel_simple_gui
{

class SelectAllSurfaceState : public GuiState
{
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
