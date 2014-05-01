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

#include <godel_surface_detection/detection/surface_detection.h>
#include <godel_surface_detection/interactive/interactive_surface_server.h>
#include <godel_surface_detection/scan/robot_scan.h>

#include <ui_robot_blending_plugin.h>
#include <ui_robot_scan_configuration.h>
#include <ui_pose_widget.h>
#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>


namespace godel_plugins
{
namespace widgets {

const std::string ROBOT_SCAN_PATH_PREVIEW_TOPIC = "robot_scan_path_preview";

class PoseWidget: public QWidget
{
Q_OBJECT
public:
	PoseWidget(QWidget *parent = NULL);

	void set_values(const tf::Transform &t);
	tf::Transform get_values();

protected:

	Ui::PoseWidget ui_;
};

class RobotScanConfigWidget: public QMainWindow
{

private:

	typedef boost::shared_ptr<godel_surface_detection::scan::RobotScan> RobotScanPtr;

Q_OBJECT
public:

	RobotScanConfigWidget(RobotScanPtr r_ptr);
	void show();

Q_SIGNALS:
	void parameters_changed();

protected:

	void init();
	void update_parameters();
	void save_parameters();

protected Q_SLOTS:

	void accept_changes_handler();
	void cancel_changes_handler();

protected:

	Ui::RobotScanConfigWindow ui_;
	RobotScanPtr robot_scan_ptr_;
	PoseWidget *world_to_obj_pose_widget_;
	PoseWidget *tcp_to_cam_pose_widget_;
};

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
	void run_scan_and_detect();
	void save_robot_scan_parameters();

protected Q_SLOTS:

	void scan_button_handler();
	void update_handler();
	void increase_tab_index_handler();
	void decrease_tab_index_handler();
	void selection_changed_handler();
	void select_all_handler();
	void deselect_all_handler();
	void hide_all_handler();
	void show_all_handler();
	void more_options_handler();
	void parameters_changed_handler();
	void preview_path_handler();

protected:
	Ui::RobotBlendingWidget ui_;
	RobotScanConfigWidget *config_window_;
	std::string param_ns_;
	godel_surface_detection::scan::RobotScan robot_scan_;
	godel_surface_detection::detection::SurfaceDetection surf_detect_;
	godel_surface_detection::interactive::InteractiveSurfaceServer surf_server_;
};

} /* namespace widgets */
}
#endif /* ROBOT_BLENDING_WIDGET_H_ */
