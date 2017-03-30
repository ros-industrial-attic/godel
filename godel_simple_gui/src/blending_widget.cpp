#include <ros/console.h>
#include "godel_msgs/SurfaceBlendingParameters.h"
#include "godel_simple_gui/blending_widget.h"
#include "godel_simple_gui/states/scan_teach_state.h"
#include "ui_blending_widget.h"

const std::string SURFACE_BLENDING_PARAMETERS_SERVICE = "surface_blending_parameters";
const static std::string SELECT_MOTION_PLAN_ACTION_SERVER_NAME = "select_motion_plan_as";

godel_simple_gui::BlendingWidget::BlendingWidget(QWidget* parent)
    : QWidget(parent),
      active_state_(NULL),
      select_motion_plan_action_client_(SELECT_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  // UI setup
  ui_ = new Ui::BlendingWidget;
  ui_->setupUi(this);

  options_ = new OptionsSubmenu();
  options_->hide();

  // Starts in scan teach state
  changeState(new ScanTeachState());

  // Wire in buttons
  connect(ui_->pushButtonNext, SIGNAL(clicked()), this, SLOT(onNextButton()));
  connect(ui_->pushButtonBack, SIGNAL(clicked()), this, SLOT(onBackButton()));
  connect(ui_->pushButtonReset, SIGNAL(clicked()), this, SLOT(onResetButton()));
  connect(ui_->pushButtonOptions, SIGNAL(clicked()), this, SLOT(onOptionsButton()));

  // Wire in Option signals
  connect(options_, SIGNAL(saveRequested()), this, SLOT(onOptionsSave()));

  // Connect to ROS services
  loadParameters();

  // Start Service Client
  ros::NodeHandle nh;
  surface_blending_parameters_client_ =
      nh.serviceClient<godel_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS_SERVICE);
}

godel_simple_gui::BlendingWidget::~BlendingWidget()
{
  delete active_state_;
  delete options_;
}

void godel_simple_gui::BlendingWidget::setText(const std::string& txt)
{
  ui_->textEditStatus->setPlainText(QString::fromStdString(txt));
}

void godel_simple_gui::BlendingWidget::appendText(const std::string& txt)
{
  ui_->textEditStatus->moveCursor(QTextCursor::End);
  ui_->textEditStatus->insertPlainText(QString::fromStdString(txt));
  ui_->textEditStatus->moveCursor(QTextCursor::End);
}

void godel_simple_gui::BlendingWidget::onNextButton() { active_state_->onNext(*this); }

void godel_simple_gui::BlendingWidget::onBackButton() { active_state_->onBack(*this); }

void godel_simple_gui::BlendingWidget::onResetButton() { active_state_->onReset(*this); }

void godel_simple_gui::BlendingWidget::onOptionsButton() { options_->show(); }

void godel_simple_gui::BlendingWidget::onOptionsSave()
{
  ROS_INFO_STREAM("Save Options Called");
  godel_msgs::SurfaceBlendingParameters msg;
  msg.request.action = godel_msgs::SurfaceBlendingParameters::Request::SAVE_PARAMETERS;
  msg.request.surface_detection = options_->surfaceDetectionParams();
  msg.request.path_params = options_->pathPlanningParams();
  msg.request.robot_scan = options_->robotScanParams();
  msg.request.scan_plan = options_->scanParams();

  if (!surface_blending_parameters_client_.call(msg.request, msg.response))
    ROS_WARN_STREAM("Could not complete service call to save parameters!");
}

void godel_simple_gui::BlendingWidget::changeState(GuiState* new_state)
{
  // Don't transition to a null new state
  if (!new_state)
    return;

  if (active_state_)
  {
    active_state_->onExit(*this);
    delete active_state_;
  }

  active_state_ = new_state;
  connect(new_state, SIGNAL(newStateAvailable(GuiState*)), this, SLOT(changeState(GuiState*)));

  new_state->onStart(*this);
}

void godel_simple_gui::BlendingWidget::setButtonsEnabled(bool enabled)
{
  ui_->pushButtonNext->setEnabled(enabled);
  ui_->pushButtonBack->setEnabled(enabled);
  ui_->pushButtonReset->setEnabled(enabled);
}

void godel_simple_gui::BlendingWidget::loadParameters()
{
  godel_msgs::SurfaceBlendingParameters srv;
  srv.request.action = srv.request.GET_CURRENT_PARAMETERS;
  ros::ServiceClient param_client =
      nodeHandle().serviceClient<godel_msgs::SurfaceBlendingParameters>(
          "surface_blending_parameters");

  setButtonsEnabled(false);
  param_client.waitForExistence();

  if (param_client.call(srv))
  {
    this->options().setRobotScanParams(srv.response.robot_scan);
    this->options().setSurfaceDetectionParams(srv.response.surface_detection);
    this->options().setPathPlanningParams(srv.response.path_params);
    this->options().setScanParams(srv.response.scan_plan);
  }
  else
  {
    ROS_WARN_STREAM("Unable to fetch blending parameters");
  }
  setButtonsEnabled(true);
}

void godel_simple_gui::BlendingWidget::showStatusWindow()
{
  ui_->stackedWidget->setCurrentWidget(ui_->status_widget);
}

void godel_simple_gui::BlendingWidget::showPlanListWidget()
{
  ui_->stackedWidget->setCurrentWidget(ui_->plan_widget);
}

void godel_simple_gui::BlendingWidget::addPlans(const std::vector<std::string> &plan_names)
{
  ui_->plan_list_widget->clear();
  for(const auto& plan : plan_names)
  {
    QListWidgetItem* item = new QListWidgetItem();
    item->setText(QString::fromStdString(plan));
    ui_->plan_list_widget->addItem(item);
  }
}

void godel_simple_gui::BlendingWidget::setLabelText(const std::string& txt)
{
  ui_->statusLabel->setText( QString::fromStdString(txt));
}

std::vector<std::string> godel_simple_gui::BlendingWidget::getPlanNames()
{
  std::vector<std::string> selected_plans;
  selected_plans.push_back(ui_->plan_list_widget->currentItem()->text().toStdString());
  return selected_plans;
}

void godel_simple_gui::BlendingWidget::sendGoal(const godel_msgs::SelectMotionPlanActionGoal& goal)
{
  select_motion_plan_action_client_.sendGoal(goal.goal);
}

void godel_simple_gui::BlendingWidget::sendGoalAndWait(const godel_msgs::SelectMotionPlanActionGoal& goal)
{
  ros::Duration timeout = ros::Duration(60);
  select_motion_plan_action_client_.sendGoalAndWait(goal.goal, timeout, timeout);
}

bool godel_simple_gui::BlendingWidget::planSelectionEmpty()
{
  return (ui_->plan_list_widget->currentItem() == NULL);
}
