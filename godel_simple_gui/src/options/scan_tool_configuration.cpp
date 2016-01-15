#include "godel_simple_gui/options/scan_tool_configuration.h"
#include "ui_scan_tool_configuration.h"

godel_simple_gui::ScanPlanConfigWidget::ScanPlanConfigWidget(godel_msgs::ScanPlanParameters params)
    : params_(params)
{
  ui_ = new Ui::ScanToolConfigWindow();
  ui_->setupUi(this);

  connect(ui_->PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_->PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_->PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  // Populate the avaiable scan-scoring methods
  ui_->ComboBoxQualityMetric->addItems(quality_metric_list);
}

QStringList godel_simple_gui::ScanPlanConfigWidget::quality_metric_list = QStringList() << "rms";

void godel_simple_gui::ScanPlanConfigWidget::update_display_fields()
{
  ui_->LineEditScanWidth->setText(QString::number(params_.scan_width));
  ui_->LineEditTravelSpeed->setText(QString::number(params_.traverse_spd));
  ui_->LineEditApproachDistance->setText(QString::number(params_.approach_distance));
  ui_->LineEditMargin->setText(QString::number(params_.margin));
  ui_->LineEditOverlap->setText(QString::number(params_.overlap));

  ui_->ComboBoxQualityMetric->setCurrentIndex(get_quality_combobox_index());
  ui_->LineEditWindowWidth->setText(QString::number(params_.window_width));
  ui_->LineEditMinQAValue->setText(QString::number(params_.min_qa_value));
  ui_->LineEditMaxQAValue->setText(QString::number(params_.max_qa_value));
}

void godel_simple_gui::ScanPlanConfigWidget::update_internal_fields()
{
  params_.scan_width = ui_->LineEditScanWidth->text().toDouble();
  params_.traverse_spd = ui_->LineEditTravelSpeed->text().toDouble();
  params_.approach_distance = ui_->LineEditApproachDistance->text().toDouble();
  params_.overlap = ui_->LineEditOverlap->text().toDouble();
  params_.margin = ui_->LineEditMargin->text().toDouble();

  params_.quality_metric = get_scan_method_enum_value();
  params_.window_width = ui_->LineEditWindowWidth->text().toDouble();
  params_.min_qa_value = ui_->LineEditMinQAValue->text().toDouble();
  params_.max_qa_value = ui_->LineEditMaxQAValue->text().toDouble();
}

int godel_simple_gui::ScanPlanConfigWidget::get_quality_combobox_index()
{
  switch (params_.quality_metric)
  {
  case godel_msgs::ScanPlanParameters::METHOD_RMS:
    return 0;
  default:
    return -1;
  }
}

int godel_simple_gui::ScanPlanConfigWidget::get_scan_method_enum_value()
{
  switch (ui_->ComboBoxQualityMetric->currentIndex())
  {
  case 0:
    return godel_msgs::ScanPlanParameters::METHOD_RMS;
  default:
    return -1;
  }
}
