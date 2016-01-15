#ifndef SIMPLE_GUI_BLENDING_WIDGET_H
#define SIMPLE_GUI_BLENDING_WIDGET_H

#include <QWidget>
#include <ros/ros.h>

#include "godel_simple_gui/gui_state.h"
#include "godel_simple_gui/options_submenu.h"

namespace Ui
{
class BlendingWidget;
}

namespace godel_simple_gui
{

class BlendingWidget : public QWidget
{
  Q_OBJECT
public:
  BlendingWidget(QWidget* parent = 0);

  virtual ~BlendingWidget();

  // Interface for the states to interact with
  void setText(const std::string& txt);
  void setButtonsEnabled(bool enabled);

  ros::NodeHandle& nodeHandle() { return nh_; }

  OptionsSubmenu& options() { return *options_; }

protected:
  void loadParameters();

protected Q_SLOTS:
  // Button Handlers
  void onNextButton();
  void onBackButton();
  void onResetButton();
  void onOptionsButton();

  void onOptionsSave();

  // State Change
  void changeState(GuiState* new_state);

protected:
  // UI
  Ui::BlendingWidget* ui_;
  OptionsSubmenu* options_;

  // Options Subwindow

  // ROS specific stuff
  ros::NodeHandle nh_;

  // Current state
  GuiState* active_state_;
};

/**
 * Works in modes:
 * 1. Scan Teach Mode (Waiting to Scan)
 * 2. Scanning
 * 3. Surface Select Mode (Select Surfaces)
 * 4. Planning
 * 5. Waiting to Preview (Next...)
 * 6. Previewing
 * 7. Waiting to Execute (Next...)
 *
 * And back to the start
 */
}

#endif
