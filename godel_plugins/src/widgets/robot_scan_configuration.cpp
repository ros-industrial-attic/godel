#include <godel_plugins/widgets/robot_scan_configuration.h>

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

// Pose Widget
godel_plugins::PoseWidget::PoseWidget(QWidget* parent) : QWidget(parent)
{
  ui_.setupUi(this);
  set_values(tf::Transform::getIdentity());
}

void godel_plugins::PoseWidget::set_values(const geometry_msgs::Pose& p)
{
  tf::Transform t;
  tf::poseMsgToTF(p, t);
  set_values(t);
}

void godel_plugins::PoseWidget::set_values(const tf::Transform& t)
{
  tf::Vector3 p = t.getOrigin();
  tfScalar rx, ry, rz;
  t.getBasis().getRPY(rx, ry, rz, 1);
  ui_.LineEditX->setText(QString::number(p.x()));
  ui_.LineEditY->setText(QString::number(p.y()));
  ui_.LineEditZ->setText(QString::number(p.z()));
  ui_.LineEditRx->setText(QString::number(RAD2DEG(rx)));
  ui_.LineEditRy->setText(QString::number(RAD2DEG(ry)));
  ui_.LineEditRz->setText(QString::number(RAD2DEG(rz)));
}

tf::Transform godel_plugins::PoseWidget::get_values()
{
  double x, y, z, rx, ry, rz;
  x = ui_.LineEditX->text().toDouble();
  y = ui_.LineEditY->text().toDouble();
  z = ui_.LineEditZ->text().toDouble();
  rx = DEG2RAD(ui_.LineEditRx->text().toDouble());
  ry = DEG2RAD(ui_.LineEditRy->text().toDouble());
  rz = DEG2RAD(ui_.LineEditRz->text().toDouble());

  // create transform
  tf::Vector3 p(x, y, z);
  tf::Quaternion q;
  q.setRPY(rx, ry, rz);

  return tf::Transform(q, p);
}

// Scan Config Widget
godel_plugins::RobotScanConfigWidget::RobotScanConfigWidget(
    const godel_msgs::RobotScanParameters& params)
    : params_(params)
{
  ui_.setupUi(this);

  world_to_obj_pose_widget_ = new PoseWidget(ui_.PoseWidgetWorldToObj);
  tcp_to_cam_pose_widget_ = new PoseWidget(ui_.PoseWidgetTcpToCam);

  connect(ui_.PushButtonAccept, SIGNAL(clicked()), this, SLOT(accept_changes_handler()));
  connect(ui_.PushButtonCancel, SIGNAL(clicked()), this, SLOT(cancel_changes_handler()));
  connect(ui_.PushButtonSave, SIGNAL(clicked()), this, SLOT(save_changes_handler()));
}

void godel_plugins::RobotScanConfigWidget::update_gui_fields()
{
  ui_.SpinBoxNumScans->setValue(params_.num_scan_points);
  ui_.LineEditCamTilt->setText(QString::number(RAD2DEG(params_.cam_tilt_angle)));
  ui_.LineEditCameraXoffset->setText(QString::number(params_.cam_to_obj_xoffset));
  ui_.LineEditCameraZoffset->setText(QString::number(params_.cam_to_obj_zoffset));
  ui_.LineEditSweepAngleStart->setText(QString::number(RAD2DEG(params_.sweep_angle_start)));
  ui_.LineEditSweepAngleEnd->setText(QString::number(RAD2DEG(params_.sweep_angle_end)));
  ui_.LineEditReachablePointRatio->setText(QString::number(params_.reachable_scan_points_ratio));
  ui_.LineEditScanTopic->setText(QString::fromStdString(params_.scan_topic));
  ui_.LineEditScanTargetFrame->setText(QString::fromStdString(params_.scan_target_frame));
  ui_.LineEditWorldFrame->setText(QString::fromStdString(params_.world_frame));
  ui_.LineEditTcpFrame->setText(QString::fromStdString(params_.tcp_frame));
  ui_.LineEditGroupName->setText(QString::fromStdString(params_.group_name));
  ui_.CheckBoxStopOnPlanningError->setChecked(params_.stop_on_planning_error);

  world_to_obj_pose_widget_->set_values(params_.world_to_obj_pose);
  tcp_to_cam_pose_widget_->set_values(params_.tcp_to_cam_pose);
}

void godel_plugins::RobotScanConfigWidget::update_internal_values()
{
  params_.num_scan_points = ui_.SpinBoxNumScans->value();
  params_.cam_tilt_angle = DEG2RAD(ui_.LineEditCamTilt->text().toDouble());
  params_.cam_to_obj_xoffset = ui_.LineEditCameraXoffset->text().toDouble();
  params_.cam_to_obj_zoffset = ui_.LineEditCameraZoffset->text().toDouble();
  params_.sweep_angle_start = DEG2RAD(ui_.LineEditSweepAngleStart->text().toDouble());
  params_.sweep_angle_end = DEG2RAD(ui_.LineEditSweepAngleEnd->text().toDouble());
  params_.reachable_scan_points_ratio = ui_.LineEditReachablePointRatio->text().toDouble();
  params_.scan_topic = ui_.LineEditScanTopic->text().toStdString();
  params_.scan_target_frame = ui_.LineEditScanTargetFrame->text().toStdString();
  params_.world_frame = ui_.LineEditWorldFrame->text().toStdString();
  params_.tcp_frame = ui_.LineEditTcpFrame->text().toStdString();
  params_.group_name = ui_.LineEditGroupName->text().toStdString();
  params_.stop_on_planning_error = ui_.CheckBoxStopOnPlanningError->isChecked();

  tf::poseTFToMsg(world_to_obj_pose_widget_->get_values(), params_.world_to_obj_pose);
  tf::poseTFToMsg(tcp_to_cam_pose_widget_->get_values(), params_.tcp_to_cam_pose);
}
