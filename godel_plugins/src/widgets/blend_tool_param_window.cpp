#include "godel_plugins/widgets/blend_tool_param_window.h"

godel_plugins::BlendingPlanConfigWidget::BlendingPlanConfigWidget(
    const godel_msgs::BlendingPlanParameters& params)
    : params_(params)
{
  ui_.setupUi(this);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));
}

void godel_plugins::BlendingPlanConfigWidget::update_gui_fields()
{
  ui_.LineEditSpindleSpeed->setText(QString::number(params_.spindle_speed));
  ui_.LineEditTravelSpeed->setText(QString::number(params_.traverse_spd));
  ui_.LineEditForce->setText(QString::number(params_.tool_force));
  ui_.LineEditSpindleDiameter->setText(QString::number(params_.tool_radius));
  ui_.LineEditMargin->setText(QString::number(params_.margin));
  ui_.lineEditOverlap->setText(QString::number(params_.overlap));
}

void godel_plugins::BlendingPlanConfigWidget::update_internal_values()
{

  params_.spindle_speed = ui_.LineEditSpindleSpeed->text().toDouble();
  params_.traverse_spd = ui_.LineEditTravelSpeed->text().toDouble();
  params_.tool_force = ui_.LineEditForce->text().toDouble();
  params_.tool_radius = ui_.LineEditSpindleDiameter->text().toDouble();
  params_.overlap = ui_.lineEditOverlap->text().toDouble();
  params_.margin = ui_.LineEditMargin->text().toDouble();
}
