#include "godel_plugins/widgets/scan_tool_configuration_window.h"

godel_plugins::ScanPlanConfigWidget::ScanPlanConfigWidget(godel_msgs::ScanPlanParameters params)
  : params_(params)
{
  ui_.setupUi(this);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));

  ui_.ComboBoxQualityMetric->addItems(quality_metric_list);

}

QStringList godel_plugins::ScanPlanConfigWidget::quality_metric_list = QStringList() << "rms" << "ra";

void godel_plugins::ScanPlanConfigWidget::update_gui_fields()
{
  ui_.LineEditScanWidth->setText(QString::number(params_.scan_width));
  ui_.LineEditTravelSpeed->setText(QString::number(params_.traverse_spd));
  ui_.LineEditApproachDistance->setText(QString::number(params_.approach_distance));
  ui_.LineEditMargin->setText(QString::number(params_.margin));
  ui_.LineEditOverlap->setText(QString::number(params_.overlap));

  ui_.ComboBoxQualityMetric->setCurrentIndex(get_quality_combobox_index());
  ui_.LineEditWindowWidth->setText(QString::number(params_.window_width));
  ui_.LineEditMinQAValue->setText(QString::number(params_.min_qa_value));
  ui_.LineEditMaxQAValue->setText(QString::number(params_.max_qa_value));
}

void godel_plugins::ScanPlanConfigWidget::update_internal_values()
{

  params_.scan_width = ui_.LineEditScanWidth->text().toDouble();
  params_.traverse_spd = ui_.LineEditTravelSpeed->text().toDouble();
  params_.approach_distance = ui_.LineEditApproachDistance->text().toDouble();
  params_.overlap = ui_.LineEditOverlap->text().toDouble();
  params_.margin = ui_.LineEditMargin->text().toDouble();

  params_.quality_metric = ui_.ComboBoxQualityMetric->currentText().toStdString();
  params_.window_width = ui_.LineEditWindowWidth->text().toDouble();
  params_.min_qa_value = ui_.LineEditMinQAValue->text().toDouble();
  params_.max_qa_value = ui_.LineEditMaxQAValue->text().toDouble();

}

int godel_plugins::ScanPlanConfigWidget::get_quality_combobox_index()
{

    for (int i = 0; i != quality_metric_list.size(); i++){
        if (params_.quality_metric == quality_metric_list.at(i).toStdString()){
            return i;
        }
    }
    return -1;
}
