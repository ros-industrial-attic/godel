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
	parameters_changed_handler();

	// initializing config window
	config_window_= new RobotScanConfigWidget(boost::shared_ptr<godel_surface_detection::scan::RobotScan>(&robot_scan_));

	// setting signals and slots
	connect(config_window_,SIGNAL(parameters_changed()),this,SLOT(parameters_changed_handler()));
	connect(ui_.PushButtonMoreOptions,SIGNAL(clicked()),this,SLOT(more_options_handler()));
	connect(ui_.PushButtonScan,SIGNAL(clicked()),this,SLOT(scan_button_handler()));
	connect(ui_.PushButtonNext,SIGNAL(clicked()),this,SLOT(increase_tab_index_handler()));
	connect(ui_.PushButtonBack,SIGNAL(clicked()),this,SLOT(decrease_tab_index_handler()));
	connect(ui_.PushButtonSelectAllSurfaces,SIGNAL(clicked()),this,SLOT(select_all_handler()));
	connect(ui_.PushButtonDeselectAllSurfaces,SIGNAL(clicked()),this,SLOT(deselect_all_handler()));
	connect(ui_.PushButtonHideAllSurfaces,SIGNAL(clicked()),this,SLOT(hide_all_handler()));
	connect(ui_.PushButtonShowAllSurfaces,SIGNAL(clicked()),this,SLOT(show_all_handler()));
	connect(ui_.PushButtonPreviewPath,SIGNAL(clicked()),this,SLOT(preview_path_handler()));
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

void RobotBlendingWidget::parameters_changed_handler()
{
	ui_.LineEditSensorTopic->setText(QString::fromStdString(robot_scan_.scan_topic_));
	ui_.SpinBoxNumScans->setValue(static_cast<int>(robot_scan_.num_scan_points_));
	ui_.LineEditCamTilt->setText(QString::number(RAD_TO_DEGREES* robot_scan_.cam_tilt_angle_));
	ui_.LineEditSweepAngleStart->setText(QString::number(RAD_TO_DEGREES* robot_scan_.sweep_angle_start_));
	ui_.LineEditSweepAngleEnd->setText(QString::number(RAD_TO_DEGREES* robot_scan_.sweep_angle_end_));
}

void RobotBlendingWidget::preview_path_handler()
{
	// saving parameters
	save_robot_scan_parameters();

	// publish path
	robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);

	ROS_INFO_STREAM("Publish path preview to 'geometry_msgs/PoseArray' topic "<<ROBOT_SCAN_PATH_PREVIEW_TOPIC);
}

void RobotBlendingWidget::more_options_handler()
{
	save_robot_scan_parameters();
	config_window_->show();
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
	// disable gui
	ui_.TabWidget->setEnabled(false);

	// saving parameters
	save_robot_scan_parameters();

	// publishing scan path preview
	robot_scan_.publish_scan_poses(ROBOT_SCAN_PATH_PREVIEW_TOPIC);

	// clear all results
	surf_detect_.clear_results();


	ROS_INFO_STREAM("Starting scan");
	int scans_completed = robot_scan_.scan(false);
	surf_server_.remove_all_surfaces();
	if(scans_completed > 0)
	{
		ROS_INFO_STREAM("Scan points reached "<<scans_completed);
		if(surf_detect_.find_surfaces())
		{
			// adding markers to server
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
	ui_.TabWidgetCreateLib->setCurrentIndex(1);
}

void RobotBlendingWidget::save_robot_scan_parameters()
{
	robot_scan_.num_scan_points_ = ui_.SpinBoxNumScans->value();
	robot_scan_.cam_tilt_angle_ = ui_.LineEditCamTilt->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.sweep_angle_start_ = ui_.LineEditSweepAngleStart->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.sweep_angle_end_ = ui_.LineEditSweepAngleEnd->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_.scan_topic_ = ui_.LineEditSensorTopic->text().toStdString();
}

RobotScanConfigWidget::RobotScanConfigWidget(RobotScanPtr r_ptr)
{
	robot_scan_ptr_ = r_ptr;
	init();
	update_parameters();
}

void RobotScanConfigWidget::show()
{
	update_parameters();
	QMainWindow::show();
}

void RobotScanConfigWidget::init()
{

	ui_.setupUi(this);
	world_to_obj_pose_widget_ = new PoseWidget(ui_.PoseWidgetWorldToObj);
	tcp_to_cam_pose_widget_ = new PoseWidget(ui_.PoseWidgetTcpToCam);

	// setting signals and slots
	connect(ui_.PushButtonAccept,SIGNAL(clicked()),this,SLOT(accept_changes_handler()));
	connect(ui_.PushButtonCancel,SIGNAL(clicked()),this,SLOT(cancel_changes_handler()));
}

void RobotScanConfigWidget::accept_changes_handler()
{
	save_parameters();
	Q_EMIT parameters_changed();
	hide();
}

void RobotScanConfigWidget::cancel_changes_handler()
{
	hide();
}

void RobotScanConfigWidget::update_parameters()
{
	ui_.SpinBoxNumScans->setValue(robot_scan_ptr_->num_scan_points_);
	ui_.LineEditCamTilt->setText(QString::number(RAD2DEG(robot_scan_ptr_->cam_tilt_angle_)));
	ui_.LineEditCameraXoffset->setText(QString::number(robot_scan_ptr_->cam_to_obj_xoffset_));
	ui_.LineEditCameraZoffset->setText(QString::number(robot_scan_ptr_->cam_to_obj_zoffset_));
	ui_.LineEditSweepAngleStart->setText(QString::number(RAD2DEG(robot_scan_ptr_->sweep_angle_start_)));
	ui_.LineEditSweepAngleEnd->setText(QString::number(RAD2DEG(robot_scan_ptr_->sweep_angle_end_)));
	ui_.LineEditReachablePointRatio->setText(QString::number(robot_scan_ptr_->reachable_scan_points_ratio_));
	ui_.LineEditScanTopic->setText(QString::fromStdString(robot_scan_ptr_->scan_topic_));
	ui_.LineEditScanTargetFrame->setText(QString::fromStdString(robot_scan_ptr_->scan_target_frame_));
	ui_.LineEditWorldFrame->setText(QString::fromStdString(robot_scan_ptr_->world_frame_));
	ui_.LineEditTcpFrame->setText(QString::fromStdString(robot_scan_ptr_->tcp_frame_));
	ui_.LineEditGroupName->setText(QString::fromStdString(robot_scan_ptr_->group_name_));
	ui_.CheckBoxStopOnPlanningError->setChecked(robot_scan_ptr_->stop_on_planning_error_);

	world_to_obj_pose_widget_->set_values(robot_scan_ptr_->world_to_obj_pose_);
	tcp_to_cam_pose_widget_->set_values(robot_scan_ptr_->tcp_to_cam_pose_);

}

void RobotScanConfigWidget::save_parameters()
{
	robot_scan_ptr_->num_scan_points_= ui_.SpinBoxNumScans->value();
	robot_scan_ptr_->cam_tilt_angle_ = DEG2RAD(ui_.LineEditCamTilt->text().toDouble());
	robot_scan_ptr_->cam_to_obj_xoffset_ = ui_.LineEditCameraXoffset->text().toDouble();
	robot_scan_ptr_->cam_to_obj_zoffset_ = ui_.LineEditCameraZoffset->text().toDouble();
	robot_scan_ptr_->sweep_angle_start_ = DEG2RAD(ui_.LineEditSweepAngleStart->text().toDouble());
	robot_scan_ptr_->sweep_angle_end_ = DEG2RAD(ui_.LineEditSweepAngleEnd->text().toDouble());
	robot_scan_ptr_->reachable_scan_points_ratio_ = ui_.LineEditReachablePointRatio->text().toDouble();
	robot_scan_ptr_->scan_topic_ = ui_.LineEditScanTopic->text().toStdString();
	robot_scan_ptr_->scan_target_frame_ = ui_.LineEditScanTargetFrame->text().toStdString();
	robot_scan_ptr_->world_frame_ = ui_.LineEditWorldFrame->text().toStdString();
	robot_scan_ptr_->tcp_frame_ = ui_.LineEditTcpFrame->text().toStdString();
	robot_scan_ptr_->group_name_ = ui_.LineEditGroupName->text().toStdString();
	robot_scan_ptr_->stop_on_planning_error_ = ui_.CheckBoxStopOnPlanningError->isChecked();

	robot_scan_ptr_->world_to_obj_pose_ = world_to_obj_pose_widget_->get_values();
	robot_scan_ptr_->tcp_to_cam_pose_ = tcp_to_cam_pose_widget_->get_values();

}

PoseWidget::PoseWidget(QWidget *parent):
		QWidget(parent)
{
	ui_.setupUi(this);
	set_values(tf::Transform::getIdentity());
}

void PoseWidget::set_values(const tf::Transform& t)
{
	tf::Vector3 p = t.getOrigin();
	tfScalar rx,ry,rz;
	t.getBasis().getRPY(rx,ry,rz,1);
	ui_.LineEditX->setText(QString::number(p.x()));
	ui_.LineEditY->setText(QString::number(p.y()));
	ui_.LineEditZ->setText(QString::number(p.z()));
	ui_.LineEditRx->setText(QString::number(RAD2DEG(rx)));
	ui_.LineEditRy->setText(QString::number(RAD2DEG(ry)));
	ui_.LineEditRz->setText(QString::number(RAD2DEG(rz)));
}

tf::Transform PoseWidget::get_values()
{
	double x,y,z,rx,ry,rz;
	x = ui_.LineEditX->text().toDouble();
	y = ui_.LineEditY->text().toDouble();
	z = ui_.LineEditZ->text().toDouble();
	rx = DEG2RAD(ui_.LineEditRx->text().toDouble());
	ry = DEG2RAD(ui_.LineEditRy->text().toDouble());
	rz = DEG2RAD(ui_.LineEditRz->text().toDouble());

	// create transform
	tf::Vector3 p(x,y,z);
	tf::Quaternion q;
	q.setRPY(rx,ry,rz);

	return tf::Transform(q,p);
}


} /* namespace widgets */

}
