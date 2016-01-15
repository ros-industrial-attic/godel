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

#ifndef ROBOT_BLENDING_PANEL_H_
#define ROBOT_BLENDING_PANEL_H_

#include <ros/ros.h>
#include <godel_plugins/widgets/robot_blending_widget.h>
#include <rviz/panel.h>

namespace godel_plugins
{
namespace rviz_panels
{

class RobotBlendingPanel : public rviz::Panel
{
  Q_OBJECT
public:
  RobotBlendingPanel(QWidget* parent = 0);
  virtual ~RobotBlendingPanel();

  virtual void onInitialize();

protected Q_SLOTS:

  // rviz::Panel virtual functions
  virtual void load(const rviz::Config& config);
  virtual void save(rviz::Config config) const;

protected:
  QWidget* widget_;
};

} /* namespace rviz_panels */
} /* namespace godel_plugins */
#endif /* ROBOT_BLENDING_PANEL_H_ */
