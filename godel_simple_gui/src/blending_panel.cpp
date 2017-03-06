#include "godel_simple_gui/blending_panel.h"

#include <ros/console.h>

#include "godel_simple_gui/blending_widget.h"

#include <QVBoxLayout>

godel_simple_gui::BlendingPanel::BlendingPanel(QWidget* parent) : rviz::Panel(parent)
{
  ROS_INFO("Loaded simple blending panel");

  QVBoxLayout* layout = new QVBoxLayout(this);
  widget_ = new BlendingWidget();
  layout->addWidget(widget_);
  setLayout(layout);
}

godel_simple_gui::BlendingPanel::~BlendingPanel() {}

void godel_simple_gui::BlendingPanel::onInitialize()
{
  ROS_INFO("Initializng simple blending panel");
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(godel_simple_gui::BlendingPanel, rviz::Panel)
