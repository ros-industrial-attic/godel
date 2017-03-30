#ifndef SIMPLE_GUI_BLENDING_WIDGET_H
#define SIMPLE_GUI_BLENDING_WIDGET_H

#include <QWidget>
#include <ros/ros.h>
#include "godel_simple_gui/gui_state.h"
#include "godel_simple_gui/options_submenu.h"
#include "actionlib/client/simple_action_client.h"
#include "godel_msgs/SelectMotionPlanAction.h"
#include "godel_msgs/SelectMotionPlanActionGoal.h"

namespace Ui
{
class BlendingWidget;
}

namespace godel_simple_gui
{
/**
 * @brief The BlendingWidget class works in states:
 * 1. Scan Teach State (User selects next)
 * 2. Scanning State (Robot Scans)
 * 3. Surface Select State (User selects surfaces)
 * 4. Planning State (Godel path plans)
 * 5. Select Plans State (User selects plans)
 * 6. Simulating State (Simulated robot executes motion)
 * 7. Waiting to Execute State (User selects next)
 * 8. Execute State (Simulated robot executes motion)
 * 9. go to step 7.
 */
class BlendingWidget : public QWidget
{
  Q_OBJECT
public:
  BlendingWidget(QWidget* parent = 0);

  virtual ~BlendingWidget();

  // Interface for the states to interact with
  void setText(const std::string& txt);
  void appendText(const std::string& txt);
  void setButtonsEnabled(bool enabled);
  void showStatusWindow();
  void showPlanListWidget();
  void clearPlanList();
  void addPlans(const std::vector<std::string>& plan_names);
  void setLabelText(const std::string& txt);
  std::vector<std::string> getPlanNames();
  void sendGoal(const godel_msgs::SelectMotionPlanActionGoal& goal);
  void sendGoalAndWait(const godel_msgs::SelectMotionPlanActionGoal& goal);
  bool planSelectionEmpty();

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

  // ROS specific stuff
  ros::NodeHandle nh_;

  // Current state
  GuiState* active_state_;
  ros::ServiceClient surface_blending_parameters_client_;
  actionlib::SimpleActionClient<godel_msgs::SelectMotionPlanAction> select_motion_plan_action_client_;

};
}

#endif
