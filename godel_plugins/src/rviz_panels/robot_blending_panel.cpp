/*
        Copyright Mar 13, 2014 Southwest Research Institute

        Licensed under the Apache License, Version 2.0 (the "License");
        you may not use this file except in compliance with the License.
        You may obtain a copy of the License at

                http://www.apache.org/licenses/LICENSE-2.0

        Unless required by applicable law or agreed to in writing, software
        distributed under the License is distributed on an "AS IS" BASIS,
        WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
        See the License for the specific language governing permissions and
        limitations under the License.
*/

#include "godel_plugins/rviz_panels/robot_blending_panel.h"

namespace godel_plugins
{
namespace rviz_panels
{

RobotBlendingPanel::RobotBlendingPanel(QWidget* parent) : rviz::Panel(parent)
{
  // TODO Auto-generated constructor stub
}

RobotBlendingPanel::~RobotBlendingPanel() {}

void RobotBlendingPanel::onInitialize()
{
  // creating main layout
  ROS_INFO_STREAM("Initializing RobotBlendingPanel");
  widget_ = new widgets::RobotBlendingWidget("~");
  this->parentWidget()->resize(widget_->width(), widget_->height());
  QHBoxLayout* main_layout = new QHBoxLayout(this);
  main_layout->addWidget(widget_);
}

void RobotBlendingPanel::load(const rviz::Config& config)
{
  rviz::Panel::load(config);
  QString text_entry;
  ROS_INFO_STREAM("rviz RobotBlendingPanel reading config file");
  if (config.mapGetString("TextEntry", &text_entry))
  {
    // ROS_INFO_STREAM("Loaded TextEntry with value: "<<text_entry.toStdString());
  }
  ROS_INFO_STREAM("rviz RobotBlendingPanel Finished reading config file");
}

void RobotBlendingPanel::save(rviz::Config config) const
{
  ROS_INFO_STREAM("Saving configuration");
  rviz::Panel::save(config);
  config.mapSetValue("TextEntry", QString::fromStdString(std::string("test_field")));
}

} /* namespace rviz_panels */
} /* namespace godel_plugins */

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(godel_plugins::rviz_panels::RobotBlendingPanel, rviz::Panel)
