#include "godel_simple_gui/states/error_state.h"

#include <ros/console.h>
#include "godel_simple_gui/blending_widget.h"

#include <QMessageBox>

godel_simple_gui::ErrorState::ErrorState(const std::string& msg, GuiState* next_state)
    : msg_(msg), next_state_(next_state)
{
}

void godel_simple_gui::ErrorState::onStart(BlendingWidget& gui)
{
  gui.setText("An error was detected, please acknowledge.");
  gui.setButtonsEnabled(false);

  // Show error dialog
  window_ = new QWidget();

  QMessageBox::StandardButton reply;
  reply =
      QMessageBox::critical(window_, tr("Warning"), QString::fromStdString(msg_), QMessageBox::Ok);
  Q_EMIT newStateAvailable(next_state_);
}

void godel_simple_gui::ErrorState::onExit(BlendingWidget& gui)
{
  delete window_;
  gui.setButtonsEnabled(true);
}

// Handlers for the fixed buttons
void godel_simple_gui::ErrorState::onNext(BlendingWidget& gui) {}

void godel_simple_gui::ErrorState::onBack(BlendingWidget& gui) {}

void godel_simple_gui::ErrorState::onReset(BlendingWidget& gui) {}
