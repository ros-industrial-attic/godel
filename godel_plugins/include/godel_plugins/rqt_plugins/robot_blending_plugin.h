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

#ifndef ROBOT_BLENDING_PLUGIN_H_
#define ROBOT_BLENDING_PLUGIN_H_

#include <godel_plugins/widgets/robot_blending_widget.h>
#include <rqt_gui_cpp/plugin.h>
#include <ui_robot_blending_plugin.h>
#include <QWidget>

namespace godel_plugins
{
namespace rqt_plugins
{

class RobotBlendingPlugin : public rqt_gui_cpp::Plugin
{

public:
  static const std::string QOBJECT_NAME;

  Q_OBJECT
public:
  RobotBlendingPlugin();
  virtual ~RobotBlendingPlugin();

  virtual void initPlugin(qt_gui_cpp::PluginContext& context);
  virtual void shutdownPlugin();
  virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings,
                            qt_gui_cpp::Settings& instance_settings) const;
  virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
                               const qt_gui_cpp::Settings& instance_settings);

private:
  QWidget* widget_;
};
}
}

#endif /* ROBOT_BLENDING_PLUGIN_H_ */
