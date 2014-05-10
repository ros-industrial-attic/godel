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

#include <godel_msgs/SurfaceDetection.h>
#include <godel_msgs/SelectSurface.h>
#include <godel_msgs/SelectedSurfacesChanged.h>
#include <tf/transform_datatypes.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <ui_robot_blending_plugin.h>
#include <ui_robot_scan_configuration.h>
#include <ui_pose_widget.h>
#include <ui_surface_detection_configuration.h>
#include <QWidget>
#include <QTimer>
#include <QtConcurrentRun>
#include <QMainWindow>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif


namespace godel_plugins
{
namespace widgets {

const std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const std::string SELECT_SURFACE_SERVICE = "select_surface";
const std::string SELECTED_SURFACES_CHANGED_TOPIC = "selected_surfaces_changed";


class PoseWidget: public QWidget
{
Q_OBJECT
public:
	PoseWidget(QWidget *parent = NULL);

	void set_values(const geometry_msgs::Pose& p);
	void set_values(const tf::Transform &t);
	tf::Transform get_values();

protected:

	Ui::PoseWidget ui_;
};

class RobotScanConfigWidget: public QMainWindow
{

private:

Q_OBJECT
public:

	RobotScanConfigWidget(godel_msgs::RobotScanParameters params);
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

public:

	godel_msgs::RobotScanParameters robot_scan_parameters_;

protected:

	Ui::RobotScanConfigWindow ui_;
	PoseWidget *world_to_obj_pose_widget_;
	PoseWidget *tcp_to_cam_pose_widget_;
};

class SurfaceDetectionConfigWidget: public QMainWindow
{

private:

Q_OBJECT
public:

	SurfaceDetectionConfigWidget(godel_msgs::SurfaceDetectionParameters params);
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

public:

	godel_msgs::SurfaceDetectionParameters surface_detection_parameters_;

protected:

	Ui::SurfaceDetectionConfigWindow ui_;
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
	void surface_detection_started();
	void surface_detection_completed();
	void connect_started();
	void connect_completed();

protected:

	void init();
	void run_scan_and_detect();
	void save_robot_scan_parameters();
	bool call_select_surface_service(godel_msgs::SelectSurface::Request &req);
	bool call_surface_detection_service(godel_msgs::SurfaceDetection& s);
	void selected_surface_changed_callback(godel_msgs::SelectedSurfacesChangedConstPtr msg);

protected Q_SLOTS:

	void scan_button_handler();
	void update_handler();
	void connect_to_services();
	void increase_tab_index_handler();
	void decrease_tab_index_handler();
	void selection_changed_handler();
	void select_all_handler();
	void deselect_all_handler();
	void hide_all_handler();
	void show_all_handler();
	void scan_options_click_handler();
	void surface_options_click_handler();
	void robot_scan_params_changed_handler();
	void surface_detect_params_changed_handler();
	void preview_path_handler();
	void surface_detection_started_handler();
	void surface_detection_completed_handler();
	void connect_started_handler();
	void connect_completed_handler();

protected:
	Ui::RobotBlendingWidget ui_;
	RobotScanConfigWidget *robot_scan_config_window_;
	SurfaceDetectionConfigWidget *surface_detect_config_window_;

	ros::ServiceClient surface_detection_client_;
	ros::ServiceClient select_surface_client_;
	ros::Subscriber selected_surfaces_subs_;
	std::string param_ns_;
	godel_msgs::RobotScanParameters robot_scan_parameters_;
	godel_msgs::SurfaceDetectionParameters surf_detect_parameters_;
	godel_msgs::SurfaceDetection::Response latest_result_;
	godel_msgs::SurfaceDetection::Request latest_request_;
	godel_msgs::SelectedSurfacesChanged selected_surfaces_msg_;
};

} /* namespace widgets */
}
#endif /* ROBOT_BLENDING_WIDGET_H_ */
