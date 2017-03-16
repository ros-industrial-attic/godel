#include "godel_simple_gui/states/scan_teach_state.h"
#include "godel_simple_gui/states/scanning_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

void godel_simple_gui::ScanTeachState::onStart(BlendingWidget& gui)
{
  gui.setText("Ready to Scan\nPress Next to Continue");
  gui.showStatusWindow();
}

void godel_simple_gui::ScanTeachState::onExit(BlendingWidget& gui) {}

void godel_simple_gui::ScanTeachState::onNext(BlendingWidget& gui)
{
  Q_EMIT newStateAvailable(new ScanningState());
}

void godel_simple_gui::ScanTeachState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::ScanTeachState::onReset(BlendingWidget& gui) {}
