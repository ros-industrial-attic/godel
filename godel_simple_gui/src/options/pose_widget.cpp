#include "godel_simple_gui/options/pose_widget.h"
#include "ui_pose_widget.h"

#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

// Pose Widget
godel_simple_gui::PoseWidget::PoseWidget(QWidget* parent) : QWidget(parent)
{
  ui_ = new Ui::PoseWidget();
  ui_->setupUi(this);
  set_values(tf::Transform::getIdentity());
}

void godel_simple_gui::PoseWidget::set_values(const geometry_msgs::Pose& p)
{
  tf::Transform t;
  tf::poseMsgToTF(p, t);
  set_values(t);
}

void godel_simple_gui::PoseWidget::set_values(const tf::Transform& t)
{
  tf::Vector3 p = t.getOrigin();
  tfScalar rx, ry, rz;
  t.getBasis().getRPY(rx, ry, rz, 1);
  ui_->LineEditX->setText(QString::number(p.x()));
  ui_->LineEditY->setText(QString::number(p.y()));
  ui_->LineEditZ->setText(QString::number(p.z()));
  ui_->LineEditRx->setText(QString::number(RAD2DEG(rx)));
  ui_->LineEditRy->setText(QString::number(RAD2DEG(ry)));
  ui_->LineEditRz->setText(QString::number(RAD2DEG(rz)));
}

tf::Transform godel_simple_gui::PoseWidget::get_values()
{
  double x, y, z, rx, ry, rz;
  x = ui_->LineEditX->text().toDouble();
  y = ui_->LineEditY->text().toDouble();
  z = ui_->LineEditZ->text().toDouble();
  rx = DEG2RAD(ui_->LineEditRx->text().toDouble());
  ry = DEG2RAD(ui_->LineEditRy->text().toDouble());
  rz = DEG2RAD(ui_->LineEditRz->text().toDouble());

  // create transform
  tf::Vector3 p(x, y, z);
  tf::Quaternion q;
  q.setRPY(rx, ry, rz);

  return tf::Transform(q, p);
}