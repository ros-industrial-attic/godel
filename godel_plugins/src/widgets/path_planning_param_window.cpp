#include "godel_plugins/widgets/path_planning_param_window.h"

godel_plugins::PathPlanningConfigWidget::PathPlanningConfigWidget(
    const godel_msgs::PathPlanningParameters& params)
    : params_(params)
{
  ui_.setupUi(this);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));
}

void godel_plugins::PathPlanningConfigWidget::update_gui_fields()
{
  ui_.LineEditDiscretization->setText(QString::number(params_.discretization));
  ui_.LineEditMargin->setText(QString::number(params_.margin));
  ui_.LineEditOverlap->setText(QString::number(params_.overlap));
  ui_.LineEditScanWidth->setText(QString::number(params_.scan_width));
  ui_.LineEditToolRadius->setText(QString::number(params_.tool_radius));
  ui_.LineEditTraverseHeight->setText(QString::number(params_.traverse_height));
}

void godel_plugins::PathPlanningConfigWidget::update_internal_values()
{
  params_.discretization = ui_.LineEditDiscretization->text().toDouble();
  params_.margin = ui_.LineEditMargin->text().toDouble();
  params_.overlap = ui_.LineEditOverlap->text().toDouble();
  params_.scan_width = ui_.LineEditScanWidth->text().toDouble();
  params_.tool_radius = ui_.LineEditToolRadius->text().toDouble();
  params_.traverse_height = ui_.LineEditTraverseHeight->text().toDouble();
}
