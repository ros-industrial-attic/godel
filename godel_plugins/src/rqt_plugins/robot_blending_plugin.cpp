
/*
        Copyright Feb 10, 2014 Southwest Research Institute

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

#include <godel_plugins/rqt_plugins/robot_blending_plugin.h>
#include <pluginlib/class_list_macros.h>

namespace godel_plugins
{
namespace rqt_plugins
{

const std::string RobotBlendingPlugin::QOBJECT_NAME = "RobotBlending";

RobotBlendingPlugin::RobotBlendingPlugin() : rqt_gui_cpp::Plugin(), widget_(0)
{
  setObjectName(QString::fromStdString(QOBJECT_NAME));
}

RobotBlendingPlugin::~RobotBlendingPlugin()
{
  // TODO Auto-generated destructor stub
}

void RobotBlendingPlugin::initPlugin(qt_gui_cpp::PluginContext& context)
{
  widget_ = new widgets::RobotBlendingWidget("rqt");
  context.addWidget(widget_);
}

void RobotBlendingPlugin::shutdownPlugin() {}
void RobotBlendingPlugin::saveSettings(qt_gui_cpp::Settings& plugin_settings,
                                       qt_gui_cpp::Settings& instance_settings) const
{
}

void RobotBlendingPlugin::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                                          const qt_gui_cpp::Settings& instance_settings)
{
}
}
}

PLUGINLIB_EXPORT_CLASS(godel_plugins::rqt_plugins::RobotBlendingPlugin, rqt_gui_cpp::Plugin)
