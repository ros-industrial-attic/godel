#include "godel_simple_gui/blending_widget.h"

// UI specific
#include "ui_blending_widget.h"
// General ROS stuff
#include <ros/console.h>
#include "godel_msgs/SurfaceBlendingParameters.h"
// States
#include "godel_simple_gui/states/scan_teach_state.h"

godel_simple_gui::BlendingWidget::BlendingWidget(QWidget* parent)
    : QWidget(parent), active_state_(NULL)
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

void godel_simple_gui::BlendingWidget::onNextButton() { active_state_->onNext(*this); }

void godel_simple_gui::BlendingWidget::onBackButton() { active_state_->onBack(*this); }

void godel_simple_gui::BlendingWidget::onResetButton() { active_state_->onReset(*this); }

void godel_simple_gui::BlendingWidget::onOptionsButton() { options_->show(); }

void godel_simple_gui::BlendingWidget::onOptionsSave() {}

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
    this->options().setBlendingParams(srv.response.blending_plan);
    this->options().setScanParams(srv.response.scan_plan);
  }
  else
  {
    ROS_WARN_STREAM("Unable to fetch blending parameters");
  }
  setButtonsEnabled(true);
}
