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

	// initializing ros comm interface
	ros::NodeHandle nh("");
	surface_detection_client_ = nh.serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);
	select_surface_client_ = nh.serviceClient<godel_msgs::SelectSurface>(SELECT_SURFACE_SERVICE);
	selected_surfaces_subs_ = nh.subscribe(SELECTED_SURFACES_CHANGED_TOPIC,1,
			&RobotBlendingWidget::selected_surface_changed_callback,this);

	// initializing gui
	ui_.setupUi(this);

	// initializing config window
	robot_scan_config_window_= new RobotScanConfigWidget(robot_scan_parameters_);

	// setting signals and slots
	connect(robot_scan_config_window_,SIGNAL(parameters_changed()),this,SLOT(parameters_changed_handler()));
	connect(ui_.PushButtonMoreOptions,SIGNAL(clicked()),this,SLOT(robot_scan_options_handler()));
	connect(ui_.PushButtonScan,SIGNAL(clicked()),this,SLOT(scan_button_handler()));
	connect(ui_.PushButtonNext,SIGNAL(clicked()),this,SLOT(increase_tab_index_handler()));
	connect(ui_.PushButtonBack,SIGNAL(clicked()),this,SLOT(decrease_tab_index_handler()));
	connect(ui_.PushButtonSelectAllSurfaces,SIGNAL(clicked()),this,SLOT(select_all_handler()));
	connect(ui_.PushButtonDeselectAllSurfaces,SIGNAL(clicked()),this,SLOT(deselect_all_handler()));
	connect(ui_.PushButtonHideAllSurfaces,SIGNAL(clicked()),this,SLOT(hide_all_handler()));
	connect(ui_.PushButtonShowAllSurfaces,SIGNAL(clicked()),this,SLOT(show_all_handler()));
	connect(ui_.PushButtonPreviewPath,SIGNAL(clicked()),this,SLOT(preview_path_handler()));
	connect(this,SIGNAL(selection_changed()),this,SLOT(selection_changed_handler()));
	connect(this,SIGNAL(surface_detection_started()),this,SLOT(surface_detection_started_handler()));
	connect(this,SIGNAL(surface_detection_completed()),this,SLOT(surface_detection_completed_handler()));
	connect(this,SIGNAL(connect_started()),this,SLOT(connect_started_handler()));
	connect(this,SIGNAL(connect_completed()),this,SLOT(connect_completed_handler()));


	// setting up timer
	QTimer *timer = new QTimer(this);
	connect(timer,SIGNAL(timeout()),this,SLOT(update_handler()));
	timer->start(4000);

	// connect from a separate thread
	QFuture<void> future = QtConcurrent::run(this,&RobotBlendingWidget::connect_to_services);
	//QTimer::singleShot(4000,this,SLOT(connect_to_services()));
}

void RobotBlendingWidget::connect_to_services()
{
	// call services to get parameters
	godel_msgs::SurfaceDetection::Request req;
	godel_msgs::SurfaceDetection::Response res;
	req.action = req.GET_CURRENT_PARAMETERS;


	// disable gui
	Q_EMIT connect_started();

	// wait for services to connect
	while(ros::ok())
	{

		ROS_INFO_STREAM("rviz blending panel connecting to services");
		if(surface_detection_client_.waitForExistence(ros::Duration(2)) &&
				select_surface_client_.waitForExistence(ros::Duration(2)))
		{

			ROS_INFO_STREAM("rviz panel connected to the services '"<<surface_detection_client_.getService()<<
					"' and '"<<select_surface_client_.getService()<<"'");

			// requesting parameters
			if(surface_detection_client_.call(req,res))
			{
				robot_scan_config_window_->robot_scan_parameters_ = res.robot_scan;
				robot_scan_parameters_ = res.robot_scan;
				surf_detect_ = res.surface_detection;

				ROS_INFO_STREAM("robot scan parameters:\n"<<robot_scan_parameters_);

				parameters_changed_handler();


				// enable gui
				Q_EMIT connect_completed();

				ROS_INFO_STREAM("Call to service for parameters succeeded");
				break;
			}
			else
			{
				ROS_ERROR_STREAM("Call to service for parameters failed");
			}
		}
		else
		{
			ROS_ERROR_STREAM("rviz panel could not connect to the services '"<<surface_detection_client_.getService()<<
					"' or '"<<select_surface_client_.getService()<<"'");
		}
	}


}

bool RobotBlendingWidget::call_select_surface_service(godel_msgs::SelectSurface::Request &req)
{
	godel_msgs::SelectSurface::Response res;
	bool succeeded = select_surface_client_.call(req,res);
	if(succeeded)
	{

	}
	else
	{

	}

	return succeeded;
}

bool RobotBlendingWidget::call_surface_detection_service(godel_msgs::SurfaceDetection& s)
{
	bool succeeded = surface_detection_client_.call(s.request,s.response);
	if(succeeded)
	{

	}
	else
	{

	}

	return succeeded;
}

void RobotBlendingWidget::selected_surface_changed_callback(godel_msgs::SelectedSurfacesChangedConstPtr msg)
{
	selected_surfaces_msg_ = *msg;
	emit_signal_selection_change();
}

void RobotBlendingWidget::select_all_handler()
{
	godel_msgs::SelectSurface::Request req;
	req.action = req.SELECT_ALL;
	call_select_surface_service(req);
}

void RobotBlendingWidget::deselect_all_handler()
{
	godel_msgs::SelectSurface::Request req;
	req.action = req.DESELECT_ALL;
	call_select_surface_service(req);
}

void RobotBlendingWidget::hide_all_handler()
{
	godel_msgs::SelectSurface::Request req;
	req.action = req.HIDE_ALL;
	call_select_surface_service(req);
}

void RobotBlendingWidget::show_all_handler()
{
	godel_msgs::SelectSurface::Request req;
	req.action = req.SHOW_ALL;
	call_select_surface_service(req);
}

void RobotBlendingWidget::parameters_changed_handler()
{
	robot_scan_parameters_ = robot_scan_config_window_->robot_scan_parameters_;

	// updating entries in gui
	ui_.LineEditSensorTopic->setText(QString::fromStdString(robot_scan_parameters_.scan_topic));
	ui_.SpinBoxNumScans->setValue(static_cast<int>(robot_scan_parameters_.num_scan_points));
	ui_.LineEditCamTilt->setText(QString::number(RAD_TO_DEGREES* robot_scan_parameters_.cam_tilt_angle));
	ui_.LineEditSweepAngleStart->setText(QString::number(RAD_TO_DEGREES* robot_scan_parameters_.sweep_angle_start));
	ui_.LineEditSweepAngleEnd->setText(QString::number(RAD_TO_DEGREES* robot_scan_parameters_.sweep_angle_end));

	// request publish scan path
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.PUBLISH_SCAN_PATH;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_;
	call_surface_detection_service(s);
}

void RobotBlendingWidget::preview_path_handler()
{
	save_robot_scan_parameters();

	// request publish scan path
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.PUBLISH_SCAN_PATH;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_;
	call_surface_detection_service(s);
}

void RobotBlendingWidget::robot_scan_options_handler()
{
	save_robot_scan_parameters();
	robot_scan_config_window_->robot_scan_parameters_ = robot_scan_parameters_;
	robot_scan_config_window_->show();
}


void RobotBlendingWidget::update_handler()
{

}

void RobotBlendingWidget::selection_changed_handler()
{
	std::vector<std::string> list = selected_surfaces_msg_.selected_surfaces;

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
	Q_EMIT surface_detection_started();


	// saving parameters
	save_robot_scan_parameters();

	// creating surface detection request
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.SCAN_AND_FIND_ONLY;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_;

	if(call_surface_detection_service(s))
	{

		if(s.response.surfaces_found)
		{
			ui_.TabWidgetCreateLib->setCurrentIndex(1);
		}
	}
	else
	{

	}

	Q_EMIT surface_detection_completed();
}

void RobotBlendingWidget::surface_detection_started_handler()
{
	ui_.TabWidget->setEnabled(false);
}

void RobotBlendingWidget::surface_detection_completed_handler()
{
	ui_.TabWidget->setEnabled(true);
}

void RobotBlendingWidget::connect_started_handler()
{
	ui_.TabWidget->setEnabled(false);
}

void RobotBlendingWidget::connect_completed_hanlder()
{
	ui_.TabWidget->setEnabled(true);
}

void RobotBlendingWidget::save_robot_scan_parameters()
{
	robot_scan_parameters_.num_scan_points = ui_.SpinBoxNumScans->value();
	robot_scan_parameters_.cam_tilt_angle = ui_.LineEditCamTilt->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.sweep_angle_start = ui_.LineEditSweepAngleStart->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.sweep_angle_end = ui_.LineEditSweepAngleEnd->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.scan_topic = ui_.LineEditSensorTopic->text().toStdString();
}

RobotScanConfigWidget::RobotScanConfigWidget(godel_msgs::RobotScanParameters params)
{
	robot_scan_parameters_ = params;
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
	ui_.SpinBoxNumScans->setValue(robot_scan_parameters_.num_scan_points);
	ui_.LineEditCamTilt->setText(QString::number(RAD2DEG(robot_scan_parameters_.cam_tilt_angle)));
	ui_.LineEditCameraXoffset->setText(QString::number(robot_scan_parameters_.cam_to_obj_xoffset));
	ui_.LineEditCameraZoffset->setText(QString::number(robot_scan_parameters_.cam_to_obj_zoffset));
	ui_.LineEditSweepAngleStart->setText(QString::number(RAD2DEG(robot_scan_parameters_.sweep_angle_start)));
	ui_.LineEditSweepAngleEnd->setText(QString::number(RAD2DEG(robot_scan_parameters_.sweep_angle_end)));
	ui_.LineEditReachablePointRatio->setText(QString::number(robot_scan_parameters_.reachable_scan_points_ratio));
	ui_.LineEditScanTopic->setText(QString::fromStdString(robot_scan_parameters_.scan_topic));
	ui_.LineEditScanTargetFrame->setText(QString::fromStdString(robot_scan_parameters_.scan_target_frame));
	ui_.LineEditWorldFrame->setText(QString::fromStdString(robot_scan_parameters_.world_frame));
	ui_.LineEditTcpFrame->setText(QString::fromStdString(robot_scan_parameters_.tcp_frame));
	ui_.LineEditGroupName->setText(QString::fromStdString(robot_scan_parameters_.group_name));
	ui_.CheckBoxStopOnPlanningError->setChecked(robot_scan_parameters_.stop_on_planning_error);

	world_to_obj_pose_widget_->set_values(robot_scan_parameters_.world_to_obj_pose);
	tcp_to_cam_pose_widget_->set_values(robot_scan_parameters_.tcp_to_cam_pose);

}

void RobotScanConfigWidget::save_parameters()
{
	robot_scan_parameters_.num_scan_points = ui_.SpinBoxNumScans->value();
	robot_scan_parameters_.cam_tilt_angle= DEG2RAD(ui_.LineEditCamTilt->text().toDouble());
	robot_scan_parameters_.cam_to_obj_xoffset= ui_.LineEditCameraXoffset->text().toDouble();
	robot_scan_parameters_.cam_to_obj_zoffset= ui_.LineEditCameraZoffset->text().toDouble();
	robot_scan_parameters_.sweep_angle_start= DEG2RAD(ui_.LineEditSweepAngleStart->text().toDouble());
	robot_scan_parameters_.sweep_angle_end= DEG2RAD(ui_.LineEditSweepAngleEnd->text().toDouble());
	robot_scan_parameters_.reachable_scan_points_ratio = ui_.LineEditReachablePointRatio->text().toDouble();
	robot_scan_parameters_.scan_topic= ui_.LineEditScanTopic->text().toStdString();
	robot_scan_parameters_.scan_target_frame= ui_.LineEditScanTargetFrame->text().toStdString();
	robot_scan_parameters_.world_frame= ui_.LineEditWorldFrame->text().toStdString();
	robot_scan_parameters_.tcp_frame= ui_.LineEditTcpFrame->text().toStdString();
	robot_scan_parameters_.group_name= ui_.LineEditGroupName->text().toStdString();
	robot_scan_parameters_.stop_on_planning_error= ui_.CheckBoxStopOnPlanningError->isChecked();

	tf::poseTFToMsg(world_to_obj_pose_widget_->get_values(),robot_scan_parameters_.world_to_obj_pose);
	tf::poseTFToMsg(tcp_to_cam_pose_widget_->get_values(),robot_scan_parameters_.tcp_to_cam_pose);

}

PoseWidget::PoseWidget(QWidget *parent):
		QWidget(parent)
{
	ui_.setupUi(this);
	set_values(tf::Transform::getIdentity());
}

void PoseWidget::set_values(const geometry_msgs::Pose& p)
{
	tf::Transform t;
	tf::poseMsgToTF(p,t);
	set_values(t);
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
