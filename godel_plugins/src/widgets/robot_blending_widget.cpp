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

#include <godel_msgs/GetAvailableMotionPlans.h>
#include <godel_msgs/SelectMotionPlan.h>

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
  delete robot_scan_config_window_;
  delete surface_detect_config_window_;
}

void RobotBlendingWidget::init()
{

	// initializing ros comm interface
	ros::NodeHandle nh("");
	surface_detection_client_ = nh.serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);
	surface_blending_parameters_client_ = nh.serviceClient<godel_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS_SERVICE);
	select_surface_client_ = nh.serviceClient<godel_msgs::SelectSurface>(SELECT_SURFACE_SERVICE);
	process_plan_client_ = nh.serviceClient<godel_msgs::ProcessPlanning>(PROCESS_PATH_SERVICE);
  get_motion_plans_client_ = nh.serviceClient<godel_msgs::GetAvailableMotionPlans>(GET_AVAILABLE_MOTION_PLANS_SERVICE);
  select_motion_plan_client_ = nh.serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);

	selected_surfaces_subs_ = nh.subscribe(SELECTED_SURFACES_CHANGED_TOPIC,1,
			&RobotBlendingWidget::selected_surface_changed_callback,this);

	// initializing gui
	ui_.setupUi(this);

	// initializing config windows
	robot_scan_config_window_= new RobotScanConfigWidget(robot_scan_parameters_);
	surface_detect_config_window_ = new SurfaceDetectionConfigWidget(surf_detect_parameters_);


	// setting signals and slots
	connect(robot_scan_config_window_,SIGNAL(parameters_changed()),this,SLOT(robot_scan_params_changed_handler()));
	connect(surface_detect_config_window_,SIGNAL(parameters_changed()),this,SLOT(surface_detect_params_changed_handler()));
	connect(ui_.PushButtonMoreOptions,SIGNAL(clicked()),this,SLOT(scan_options_click_handler()));
	connect(ui_.PushButtonSurfaceOptions,SIGNAL(clicked()),this,SLOT(surface_options_click_handler()));
	connect(ui_.PushButtonScan,SIGNAL(clicked()),this,SLOT(scan_button_handler()));
	connect(ui_.PushButtonFindSurface,SIGNAL(clicked()),this,SLOT(find_surface_button_handler()));
	connect(ui_.PushButtonNext,SIGNAL(clicked()),this,SLOT(increase_tab_index_handler()));
	connect(ui_.PushButtonBack,SIGNAL(clicked()),this,SLOT(decrease_tab_index_handler()));
	connect(ui_.PushButtonSelectAllSurfaces,SIGNAL(clicked()),this,SLOT(select_all_handler()));
	connect(ui_.PushButtonDeselectAllSurfaces,SIGNAL(clicked()),this,SLOT(deselect_all_handler()));
	connect(ui_.PushButtonHideAllSurfaces,SIGNAL(clicked()),this,SLOT(hide_all_handler()));
	connect(ui_.PushButtonShowAllSurfaces,SIGNAL(clicked()),this,SLOT(show_all_handler()));
	connect(ui_.PushButtonPreviewPath,SIGNAL(clicked()),this,SLOT(preview_path_handler()));
	connect(ui_.PushButtonGeneratePaths,SIGNAL(clicked()),this,SLOT(generate_process_path_handler()));
	connect(this,SIGNAL(selection_changed()),this,SLOT(selection_changed_handler()));
	connect(this,SIGNAL(surface_detection_started()),this,SLOT(surface_detection_started_handler()));
	connect(this,SIGNAL(surface_detection_completed()),this,SLOT(surface_detection_completed_handler()));
	connect(this,SIGNAL(connect_started()),this,SLOT(connect_started_handler()));
	connect(this,SIGNAL(connect_completed()),this,SLOT(connect_completed_handler()));

  connect(ui_.pushButtonExecutePath, SIGNAL(clicked()), this, SLOT(execute_motion_plan_handler()));
  connect(ui_.pushButtonSimulatePath, SIGNAL(clicked(bool)), this, SLOT(simulate_motion_plan_handler()));

  // For trajectory execution


	// moving to first tab
	ui_.TabWidgetCreateLib->setCurrentIndex(0);

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
	godel_msgs::SurfaceBlendingParameters::Request req;
	godel_msgs::SurfaceBlendingParameters::Response res;
	req.action = req.GET_CURRENT_PARAMETERS;


	// disable gui
	Q_EMIT connect_started();

	// wait for services to connect
	while(ros::ok())
	{

		ROS_INFO_STREAM("rviz blending panel connecting to services");
		if(surface_detection_client_.waitForExistence(ros::Duration(2)) &&
				select_surface_client_.waitForExistence(ros::Duration(2)) &&
				surface_blending_parameters_client_.waitForExistence(ros::Duration(2)))
		{

			ROS_INFO_STREAM("rviz panel connected to the services '"<<surface_detection_client_.getService()<<
					"' and '"<<select_surface_client_.getService()<<"'");

			// requesting parameters
			if(surface_blending_parameters_client_.call(req,res))
			{
				robot_scan_config_window_->robot_scan_parameters_ = res.robot_scan;
				surface_detect_config_window_->surface_detection_parameters_ = res.surface_detection;
				robot_scan_parameters_ = res.robot_scan;
				surf_detect_parameters_ = res.surface_detection;
				blending_plan_parameters_ = res.blending_plan;

				// update gui elements for robot scan
				robot_scan_params_changed_handler();

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
			ROS_ERROR_STREAM("rviz panel could not connect to one or more ros services:\n\t'"<<surface_detection_client_.getService()<<
					"'\n\t'"<<select_surface_client_.getService()<<"'\n\t'"<<surface_blending_parameters_client_.getService());
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

void RobotBlendingWidget::robot_scan_params_changed_handler()
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
	s.request.surface_detection = surf_detect_parameters_;
	call_surface_detection_service(s);
}

void RobotBlendingWidget::surface_detect_params_changed_handler()
{
	surf_detect_parameters_ = surface_detect_config_window_->surface_detection_parameters_;
}

void RobotBlendingWidget::preview_path_handler()
{
	save_robot_scan_parameters();

	// request publish scan path
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.PUBLISH_SCAN_PATH;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_parameters_;
	call_surface_detection_service(s);
}

void RobotBlendingWidget::scan_options_click_handler()
{
	save_robot_scan_parameters();
	robot_scan_config_window_->robot_scan_parameters_ = robot_scan_parameters_;
	robot_scan_config_window_->show();
}

void RobotBlendingWidget::surface_options_click_handler()
{
	surface_detect_config_window_->surface_detection_parameters_ = surf_detect_parameters_;
	surface_detect_config_window_->show();
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
	QFuture<void> future = QtConcurrent::run(this,&RobotBlendingWidget::send_scan_and_find_request);
}

void RobotBlendingWidget::find_surface_button_handler()
{
	QFuture<void> future = QtConcurrent::run(this,&RobotBlendingWidget::send_find_surface_request);
}

void RobotBlendingWidget::send_find_surface_request()
{
	surface_detection_op_message_ = "FIND IN PROGRESS";

	// disable gui
	Q_EMIT surface_detection_started();

	// creating surface detection request
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.FIND_ONLY;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_parameters_;

	if(call_surface_detection_service(s) )
	{
		if(s.response.surfaces_found)
		{
			surface_detection_op_succeeded_ = true;
			surface_detection_op_message_ = "FIND COMPLETED";
		}
		else
		{
			surface_detection_op_succeeded_ = false;
			surface_detection_op_message_ = "FIND FAILED";
		}
	}
	else
	{
		surface_detection_op_succeeded_ = false;
		surface_detection_op_message_ = "SERVICE CALL FAILED";
	}

	// enable widget
	Q_EMIT surface_detection_completed();
}


void RobotBlendingWidget::send_scan_and_find_request()
{
	surface_detection_op_message_ = "SCAN & FIND IN PROGRESS";

	// disable gui
	Q_EMIT surface_detection_started();


	// saving parameters
	save_robot_scan_parameters();

	// creating surface detection request
	godel_msgs::SurfaceDetection s;
	s.request.action = s.request.SCAN_AND_FIND_ONLY;
	s.request.use_default_parameters = false;
	s.request.robot_scan = robot_scan_parameters_;
	s.request.surface_detection = surf_detect_parameters_;

	if(call_surface_detection_service(s) )
	{
		if(s.response.surfaces_found)
		{
			surface_detection_op_succeeded_ = true;
			surface_detection_op_message_ = "SCAN & FIND COMPLETED";
		}
		else
		{
			surface_detection_op_succeeded_ = false;
			surface_detection_op_message_ = "SCAN & FIND FAILED";
		}
	}
	else
	{
		surface_detection_op_succeeded_ = false;
		surface_detection_op_message_ = "SERVICE CALL FAILED";
	}

	Q_EMIT surface_detection_completed();
}

void RobotBlendingWidget::surface_detection_started_handler()
{
	ui_.LineEditOperationStatus->setText(QString::fromStdString(surface_detection_op_message_));
	ui_.TabWidget->setEnabled(false);
}

void RobotBlendingWidget::surface_detection_completed_handler()
{
	ui_.LineEditOperationStatus->setText(QString::fromStdString(surface_detection_op_message_));
	if(surface_detection_op_succeeded_)
	{
		ui_.TabWidgetCreateLib->setCurrentIndex(1);
	}
	ui_.TabWidget->setEnabled(true);
}

void RobotBlendingWidget::connect_started_handler()
{
	ui_.LineEditOperationStatus->setText("CONNECTING TO SERVICE");
	ui_.TabWidget->setEnabled(false);
}

void RobotBlendingWidget::connect_completed_handler()
{
	ui_.LineEditOperationStatus->setText("READY");
	ui_.TabWidget->setEnabled(true);
}

void RobotBlendingWidget::generate_process_path_handler()
{
	godel_msgs::ProcessPlanning process_plan;
	process_plan.request.use_default_parameters = false;
	process_plan.request.params = blending_plan_parameters_;
	process_plan.request.action = process_plan.request.GENERATE_MOTION_PLAN_AND_PREVIEW;
  ROS_INFO_STREAM("process plan request sent");
  if (process_plan_client_.call(process_plan))
  {
    std::vector<std::string> plan_names;
    request_available_motions(plan_names);
    ui_.ListPathResults->clear();
    for (std::size_t i = 0; i < plan_names.size(); ++i)
    {
      QListWidgetItem *item = new QListWidgetItem();
      item->setText(QString::fromStdString(plan_names[i]));
      ui_.ListPathResults->addItem(item);
    }

    ui_.TabWidgetCreateLib->setCurrentIndex(2);
  }
}

void RobotBlendingWidget::save_robot_scan_parameters()
{
	robot_scan_parameters_.num_scan_points = ui_.SpinBoxNumScans->value();
	robot_scan_parameters_.cam_tilt_angle = ui_.LineEditCamTilt->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.sweep_angle_start = ui_.LineEditSweepAngleStart->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.sweep_angle_end = ui_.LineEditSweepAngleEnd->text().toDouble()*DEGREES_TO_RAD;
	robot_scan_parameters_.scan_topic = ui_.LineEditSensorTopic->text().toStdString();
}

void RobotBlendingWidget::request_available_motions(std::vector<std::string> &plans)
{
  godel_msgs::GetAvailableMotionPlans srv;
  if (get_motion_plans_client_.call(srv))
  {
    plans = srv.response.names;
  }
  else
  {
    ROS_ERROR_STREAM("Could not get names from 'available motions server'");
  }
}

void RobotBlendingWidget::select_motion_plan(const std::string &name, bool simulate)
{
  godel_msgs::SelectMotionPlan srv;
  srv.request.name = name;
  srv.request.simulate = simulate;
  select_motion_plan_client_.call(srv);
}


void RobotBlendingWidget::simulate_motion_plan_handler()
{
  if (ui_.ListPathResults->currentItem() == NULL) return;
  std::string name = ui_.ListPathResults->currentItem()->text().toStdString();
  ROS_INFO_STREAM("Selected " << name << " to be simulated");
  if (!name.empty()) select_motion_plan(name, true);
}

void RobotBlendingWidget::execute_motion_plan_handler()
{
  if (ui_.ListPathResults->currentItem() == NULL) return;
  std::string name = ui_.ListPathResults->currentItem()->text().toStdString();
  ROS_INFO_STREAM("Selected " << name << " to be executed");
  if (!name.empty()) select_motion_plan(name, false);
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

SurfaceDetectionConfigWidget::SurfaceDetectionConfigWidget(godel_msgs::SurfaceDetectionParameters params)
{
	surface_detection_parameters_ = params;
	init();
	update_parameters();
}

void SurfaceDetectionConfigWidget::show()
{
	update_parameters();
	QMainWindow::show();
}

void SurfaceDetectionConfigWidget::init()
{
	ui_.setupUi(this);

	// setting signals and slots
	connect(ui_.PushButtonAccept,SIGNAL(clicked()),this,SLOT(accept_changes_handler()));
	connect(ui_.PushButtonCancel,SIGNAL(clicked()),this,SLOT(cancel_changes_handler()));
}

void SurfaceDetectionConfigWidget::update_parameters()
{
	ui_.LineEditFrameId->setText(QString::fromStdString(surface_detection_parameters_.frame_id));
	ui_.LineEditKSearch->setText(QString::number(surface_detection_parameters_.k_search));
	ui_.LineEditMarkerAlpha->setText(QString::number(surface_detection_parameters_.marker_alpha));

	ui_.LineEditStOutMean->setText(QString::number(surface_detection_parameters_.meanK));
	ui_.LineEditStOutThreshold->setText(QString::number(surface_detection_parameters_.stdv_threshold));

	ui_.LineEditRgMinClusterSize->setText(QString::number(surface_detection_parameters_.rg_min_cluster_size));
	ui_.LineEditRgMaxClusterSize->setText(QString::number(surface_detection_parameters_.rg_max_cluster_size));
	ui_.LineEditRgNeighbors->setText(QString::number(surface_detection_parameters_.rg_neightbors));
	ui_.LineEditRgSmoothnessThreshold->setText(QString::number(RAD2DEG(surface_detection_parameters_.rg_smoothness_threshold)));
	ui_.LineEditRgCurvatureThreshold->setText(QString::number(surface_detection_parameters_.rg_curvature_threshold));

	ui_.LineEditVoxelLeaf->setText(QString::number(surface_detection_parameters_.voxel_leafsize));
	ui_.LineEditTabletopSegmentationDist->setText(QString::number(surface_detection_parameters_.tabletop_seg_distance_threshold));
	ui_.CheckBoxUseTabletopSegmentation->setChecked(static_cast<bool>(surface_detection_parameters_.use_tabletop_seg));
	ui_.CheckBoxIgnoreLargestCluster->setChecked(static_cast<bool>(surface_detection_parameters_.ignore_largest_cluster));

	ui_.LineEditMlsPointDensity->setText(QString::number(surface_detection_parameters_.mls_point_density));
	ui_.LineEditMlsUpsamplingRadius->setText(QString::number(surface_detection_parameters_.mls_upsampling_radius));
	ui_.LineEditMlsSearchRadius->setText(QString::number(surface_detection_parameters_.mls_search_radius));

	ui_.LineEditTrSearchRadius->setText(QString::number(surface_detection_parameters_.tr_search_radius));
	ui_.LineEditTrMu->setText(QString::number(surface_detection_parameters_.tr_mu));
	ui_.LineEditTrNearestNeighbors->setText(QString::number(surface_detection_parameters_.tr_max_nearest_neighbors));
	ui_.LineEditTrMaxSurfaceAngle->setText(QString::number(RAD2DEG( surface_detection_parameters_.tr_max_surface_angle)));
	ui_.LineEditTrMinAngle->setText(QString::number(RAD2DEG( surface_detection_parameters_.tr_min_angle)));
	ui_.LineEditTrMaxAngle->setText(QString::number(RAD2DEG( surface_detection_parameters_.tr_max_angle)));
	ui_.CheckBoxTrNormalConsistency->setChecked(static_cast<bool>(surface_detection_parameters_.tr_normal_consistency));

	ui_.CheckBoxPaEnabled->setChecked(static_cast<bool>(surface_detection_parameters_.pa_enabled));
	ui_.LineEditPaSegMaxIterations->setText(QString::number(surface_detection_parameters_.pa_seg_max_iterations));
	ui_.LineEditPaSegDistThreshold->setText(QString::number(surface_detection_parameters_.pa_seg_dist_threshold));
	ui_.LineEditPaSACPlaneDistance->setText(QString::number(surface_detection_parameters_.pa_sac_plane_distance));
	ui_.LineEditPaKdtreeRadius->setText(QString::number(surface_detection_parameters_.pa_kdtree_radius));

}

void SurfaceDetectionConfigWidget::save_parameters()
{
	surface_detection_parameters_.frame_id = ui_.LineEditFrameId->text().toStdString();
	surface_detection_parameters_.k_search = ui_.LineEditKSearch->text().toDouble();
	surface_detection_parameters_.marker_alpha = ui_.LineEditMarkerAlpha->text().toDouble();

	surface_detection_parameters_.meanK = ui_.LineEditStOutMean->text().toDouble();
	surface_detection_parameters_.stdv_threshold = ui_.LineEditStOutThreshold->text().toDouble();

	surface_detection_parameters_.rg_min_cluster_size = ui_.LineEditRgMinClusterSize->text().toDouble();
	surface_detection_parameters_.rg_max_cluster_size = ui_.LineEditRgMaxClusterSize->text().toDouble();
	surface_detection_parameters_.rg_neightbors = ui_.LineEditRgNeighbors->text().toDouble();
	surface_detection_parameters_.rg_smoothness_threshold = DEG2RAD(ui_.LineEditRgSmoothnessThreshold->text().toDouble());
	surface_detection_parameters_.rg_curvature_threshold = ui_.LineEditRgCurvatureThreshold->text().toDouble();

	surface_detection_parameters_.voxel_leafsize = ui_.LineEditVoxelLeaf->text().toDouble();
	surface_detection_parameters_.tabletop_seg_distance_threshold = ui_.LineEditTabletopSegmentationDist->text().toDouble();
	surface_detection_parameters_.use_tabletop_seg = ui_.CheckBoxUseTabletopSegmentation->isChecked();
	surface_detection_parameters_.ignore_largest_cluster = ui_.CheckBoxIgnoreLargestCluster->isChecked();

	surface_detection_parameters_.mls_point_density = ui_.LineEditMlsPointDensity->text().toDouble();
	surface_detection_parameters_.mls_upsampling_radius = ui_.LineEditMlsUpsamplingRadius->text().toDouble();
	surface_detection_parameters_.mls_search_radius = ui_.LineEditMlsSearchRadius->text().toDouble();

	surface_detection_parameters_.tr_search_radius = ui_.LineEditTrSearchRadius->text().toDouble();
	surface_detection_parameters_.tr_mu = ui_.LineEditTrMu->text().toDouble();
	surface_detection_parameters_.tr_max_nearest_neighbors = ui_.LineEditTrNearestNeighbors->text().toDouble();
	surface_detection_parameters_.tr_max_surface_angle = DEG2RAD( ui_.LineEditTrMaxSurfaceAngle->text().toDouble());
	surface_detection_parameters_.tr_min_angle = DEG2RAD(ui_.LineEditTrMinAngle->text().toDouble());
	surface_detection_parameters_.tr_max_angle = DEG2RAD(ui_.LineEditTrMaxAngle->text().toDouble());
	surface_detection_parameters_.tr_normal_consistency = ui_.CheckBoxTrNormalConsistency->isChecked();

	surface_detection_parameters_.pa_enabled = ui_.CheckBoxPaEnabled->isChecked();
	surface_detection_parameters_.pa_seg_max_iterations = ui_.LineEditPaSegMaxIterations->text().toInt();
	surface_detection_parameters_.pa_seg_dist_threshold = ui_.LineEditPaSegDistThreshold->text().toDouble();
	surface_detection_parameters_.pa_sac_plane_distance = ui_.LineEditPaSACPlaneDistance->text().toDouble();
	surface_detection_parameters_.pa_kdtree_radius = ui_.LineEditPaKdtreeRadius->text().toDouble();
}

void SurfaceDetectionConfigWidget::accept_changes_handler()
{
	save_parameters();
	Q_EMIT parameters_changed();
	hide();
}

void SurfaceDetectionConfigWidget::cancel_changes_handler()
{
	hide();
}


} /* namespace widgets */

}
