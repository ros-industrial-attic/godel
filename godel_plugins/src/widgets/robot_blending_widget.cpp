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

#include <godel_plugins/widgets/robot_blending_widget.h>

const double RAD_TO_DEGREES = 180.0f/M_PI;
const double DEGREES_TO_RAD = M_PI/180.0f;

namespace godel_plugins
{
namespace widgets {

RobotBlendingWidget::RobotBlendingWidget(std::string ns):
		param_ns_(ns)
{
	// TODO Auto-generated constructor stub
	init();
}

RobotBlendingWidget::~RobotBlendingWidget() {
	// TODO Auto-generated destructor stub
}

void RobotBlendingWidget::init()
{
	using namespace godel_surface_detection;

	// initializing surface detector
	if(surf_detect_.load_parameters(param_ns_ + "/surface_detection") && robot_scan_.load_parameters(param_ns_ + "/robot_scan") &&
			surf_server_.load_parameters(param_ns_))
	{

		ROS_INFO_STREAM("Parameters for surface detector and server loaded successfully");
		if(surf_detect_.init() && robot_scan_.init() && surf_server_.init())
		{
			// adding callbacks
			scan::RobotScan::ScanCallback cb = boost::bind(&detection::SurfaceDetection::add_cloud,&surf_detect_,_1);
			robot_scan_.add_scan_callback(cb);
			ROS_INFO_STREAM("Initialization succeeded");
		}
		else
		{
			ROS_ERROR_STREAM("Initialization error");
		}

	}
	else
	{
		ROS_ERROR_STREAM("Parameters failed to load, using defaults");
	}

	// start server
	interactive::InteractiveSurfaceServer::SelectionCallback f =	boost::bind(
			&RobotBlendingWidget::emit_signal_selection_change,this);
	surf_server_.add_selection_callback(f);
	surf_server_.run();

	// initializing gui
	ui_.setupUi(this);
	ui_.TabWidget->setCurrentIndex(0);
	ui_.LineEditSensorTopic->setText(QString::fromStdString(robot_scan_.scan_topic_));
	ui_.SpinBoxNumScans->setValue(static_cast<int>(robot_scan_.num_scan_points_));
	ui_.LineEditCamTilt->setText(QString::number(RAD_TO_DEGREES* robot_scan_.cam_tilt_angle_));
	ui_.LineEditSweepAngleStart->setText(QString::number(RAD_TO_DEGREES* robot_scan_.sweep_angle_start_));
	ui_.LineEditSweepAngleEnd->setText(QString::number(RAD_TO_DEGREES* robot_scan_.sweep_angle_end_));

	// setting signals and slots
	connect(ui_.PushButtonMoreOptions,SIGNAL(clicked()),this,SLOT(more_options_handler()));
	connect(ui_.PushButtonScan,SIGNAL(clicked()),this,SLOT(scan_button_handler()));
	connect(ui_.PushButtonNext,SIGNAL(clicked()),this,SLOT(increase_tab_index_handler()));
	connect(ui_.PushButtonBack,SIGNAL(clicked()),this,SLOT(decrease_tab_index_handler()));
	connect(ui_.PushButtonSelectAllSurfaces,SIGNAL(clicked()),this,SLOT(select_all_handler()));
	connect(ui_.PushButtonDeselectAllSurfaces,SIGNAL(clicked()),this,SLOT(deselect_all_handler()));
	connect(ui_.PushButtonHideAllSurfaces,SIGNAL(clicked()),this,SLOT(hide_all_handler()));
	connect(ui_.PushButtonShowAllSurfaces,SIGNAL(clicked()),this,SLOT(show_all_handler()));
	connect(this,SIGNAL(selection_changed()),this,SLOT(selection_changed_handler()));


	// setting up timer
	QTimer *timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(update_handler()));
	timer->start(1000);
}

void RobotBlendingWidget::select_all_handler()
{
	surf_server_.select_all(true);
}

void RobotBlendingWidget::deselect_all_handler()
{
	surf_server_.select_all(false);
}

void RobotBlendingWidget::hide_all_handler()
{
	surf_server_.show_all(false);
}

void RobotBlendingWidget::show_all_handler()
{
	surf_server_.show_all(true);
}

void RobotBlendingWidget::more_options_handler()
{
	TestWindow *window = new TestWindow();
	window->show();
}


void RobotBlendingWidget::update_handler()
{

}

void RobotBlendingWidget::selection_changed_handler()
{
	std::vector<std::string> list;
	surf_server_.get_selected_list(list);

	ui_.ListWidgetSelectedSurfs->clear();
	if(list.size() > 0)
	{
		for(std::vector<std::string>::iterator i = list.begin(); i != list.end();i++)
		{
			QListWidgetItem *item = new QListWidgetItem();
			item->setText(QString::fromStdString(*i));
			ui_.ListWidgetSelectedSurfs->addItem(item);

		}
	}
}

void RobotBlendingWidget::increase_tab_index_handler()
{
	ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() + 1);
}

void RobotBlendingWidget::decrease_tab_index_handler()
{
	ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() - 1);
}

void RobotBlendingWidget::scan_button_handler()
{
	QFuture<void> future = QtConcurrent::run(this,&RobotBlendingWidget::run_scan_and_detect);
}

void RobotBlendingWidget::run_scan_and_detect()
{
	robot_scan_.num_scan_points_ = ui_.SpinBoxNumScans->value();
	robot_scan_.cam_tilt_angle_ = ui_.LineEditCamTilt->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.sweep_angle_start_ = ui_.LineEditSweepAngleStart->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.sweep_angle_end_ = ui_.LineEditSweepAngleEnd->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.scan_topic_ = ui_.LineEditSensorTopic->text().toStdString();

	ui_.TabWidget->setEnabled(false);
	int scans_completed = robot_scan_.scan(false);
	if(scans_completed > 0)
	{
		ROS_INFO_STREAM("Scan points reached "<<scans_completed);
		if(surf_detect_.find_surfaces())
		{
			// adding markers to server
			surf_server_.remove_all_surfaces();
			visualization_msgs::MarkerArray markers_msg = surf_detect_.get_surface_markers();
			for(int i =0;i < markers_msg.markers.size();i++)
			{
				surf_server_.add_surface(markers_msg.markers[i]);
			}
		}
	}
	else
	{
		ROS_ERROR_STREAM("Scan failed");
	}
	ui_.TabWidget->setEnabled(true);
}

} /* namespace widgets */

}
