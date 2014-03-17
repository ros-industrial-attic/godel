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

#ifndef ROBOT_BLENDING_WIDGET_H_
#define ROBOT_BLENDING_WIDGET_H_

#include <ui_robot_blending_plugin.h>
#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <QWidget>

namespace godel_plugins
{
namespace widgets {

class RobotBlendingWidget:  public QWidget
{
Q_OBJECT
public:
	RobotBlendingWidget(std::string ns="");
	virtual ~RobotBlendingWidget();

	std::string get_name()
	{
		return "RobotBlending";
	}

	int width()
	{
		return ui_.TabWidget->width();
	}

	int height()
	{
		return ui_.TabWidget->height();
	}

	void emit_signal_selection_change()
	{
		Q_EMIT selection_changed();
	}

Q_SIGNALS:
	void selection_changed();

protected:

	void init();


protected Q_SLOTS:

	void acquire_button_handler();
	void update_handler();
	void increase_tab_index_handler();
	void decrease_tab_index_handler();
	void selection_changed_handler();
	void select_all_handler();
	void deselect_all_handler();

protected:
	Ui::RobotBlendingWidget ui_;
	std::string param_ns_;
	godel_surface_detection::detection::SurfaceDetection surf_detect_;
	godel_surface_detection::interactive::InteractiveSurfaceServer surf_server_;
};

} /* namespace widgets */
}
#endif /* ROBOT_BLENDING_WIDGET_H_ */
