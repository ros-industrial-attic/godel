#include "godel_simple_gui/states/surface_select_state.h"
#include "godel_simple_gui/states/scan_teach_state.h"
#include "godel_simple_gui/states/planning_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

void godel_simple_gui::SurfaceSelectState::onStart(BlendingWidget& gui)
{
  gui.setText("Pick Surfaces With Mouse\nPress Next to Continue");
}

void godel_simple_gui::SurfaceSelectState::onExit(BlendingWidget& gui) {}

// Handlers for the fixed buttons
void godel_simple_gui::SurfaceSelectState::onNext(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new PlanningState());
}

void godel_simple_gui::SurfaceSelectState::onBack(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new ScanTeachState());
}

void godel_simple_gui::SurfaceSelectState::onReset(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new ScanTeachState());
}
