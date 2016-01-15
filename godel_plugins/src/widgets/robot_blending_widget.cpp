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

#include <QFileDialog>
#include <QInputDialog>

#include <godel_msgs/GetAvailableMotionPlans.h>
#include <godel_msgs/SelectMotionPlan.h>
#include <godel_msgs/LoadSaveMotionPlan.h>
#include <godel_msgs/RenameSurface.h>

const double RAD_TO_DEGREES = 180.0f / M_PI;
const double DEGREES_TO_RAD = M_PI / 180.0f;

namespace godel_plugins
{
namespace widgets
{

RobotBlendingWidget::RobotBlendingWidget(std::string ns) : param_ns_(ns)
{
  // TODO Auto-generated constructor stub
  init();
}

RobotBlendingWidget::~RobotBlendingWidget()
{
  delete robot_scan_config_window_;
  delete surface_detect_config_window_;
  delete robot_blend_config_window_;
  delete scan_plan_config_window_;
}

void RobotBlendingWidget::init()
{

  // initializing ros comm interface
  ros::NodeHandle nh("");
  surface_detection_client_ =
      nh.serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);
  surface_blending_parameters_client_ =
      nh.serviceClient<godel_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS_SERVICE);
  select_surface_client_ = nh.serviceClient<godel_msgs::SelectSurface>(SELECT_SURFACE_SERVICE);
  process_plan_client_ = nh.serviceClient<godel_msgs::ProcessPlanning>(PROCESS_PATH_SERVICE);
  get_motion_plans_client_ =
      nh.serviceClient<godel_msgs::GetAvailableMotionPlans>(GET_AVAILABLE_MOTION_PLANS_SERVICE);
  select_motion_plan_client_ =
      nh.serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
  load_save_motion_plan_client_ =
      nh.serviceClient<godel_msgs::LoadSaveMotionPlan>(LOAD_SAVE_MOTION_PLAN_SERVICE);
  rename_surface_client_ = nh.serviceClient<godel_msgs::RenameSurface>(RENAME_SURFACE_SERVICE);

  selected_surfaces_subs_ =
      nh.subscribe(SELECTED_SURFACES_CHANGED_TOPIC, 1,
                   &RobotBlendingWidget::selected_surface_changed_callback, this);

  // initializing gui
  ui_.setupUi(this);

  // initializing config windows
  robot_scan_config_window_ = new RobotScanConfigWidget(robot_scan_parameters_);
  surface_detect_config_window_ = new SurfaceDetectionConfigWidget(surf_detect_parameters_);

  robot_blend_config_window_ = new BlendingPlanConfigWidget(blending_plan_parameters_);
  scan_plan_config_window_ = new ScanPlanConfigWidget(scan_plan_parameters_);

  // setting signals and slots
  connect(robot_scan_config_window_, SIGNAL(parameters_changed()), this,
          SLOT(robot_scan_params_changed_handler()));
  connect(surface_detect_config_window_, SIGNAL(parameters_changed()), this,
          SLOT(surface_detect_params_changed_handler()));
  connect(ui_.PushButtonMoreOptions, SIGNAL(clicked()), this, SLOT(scan_options_click_handler()));

  connect(ui_.PushButtonBlendOptions, SIGNAL(clicked()), this, SLOT(blend_options_click_handler()));
  connect(ui_.pushButtonProfileOptions, SIGNAL(clicked()), this,
          SLOT(scan_plan_options_click_handler()));

  connect(ui_.PushButtonSurfaceOptions, SIGNAL(clicked()), this,
          SLOT(surface_options_click_handler()));
  connect(ui_.PushButtonScan, SIGNAL(clicked()), this, SLOT(scan_button_handler()));
  connect(ui_.PushButtonFindSurface, SIGNAL(clicked()), this, SLOT(find_surface_button_handler()));
  connect(ui_.PushButtonNext, SIGNAL(clicked()), this, SLOT(increase_tab_index_handler()));
  connect(ui_.PushButtonBack, SIGNAL(clicked()), this, SLOT(decrease_tab_index_handler()));
  connect(ui_.PushButtonSelectAllSurfaces, SIGNAL(clicked()), this, SLOT(select_all_handler()));
  connect(ui_.PushButtonDeselectAllSurfaces, SIGNAL(clicked()), this, SLOT(deselect_all_handler()));
  connect(ui_.PushButtonHideAllSurfaces, SIGNAL(clicked()), this, SLOT(hide_all_handler()));
  connect(ui_.PushButtonShowAllSurfaces, SIGNAL(clicked()), this, SLOT(show_all_handler()));
  connect(ui_.PushButtonPreviewPath, SIGNAL(clicked()), this, SLOT(preview_path_handler()));
  connect(ui_.PushButtonGeneratePaths, SIGNAL(clicked()), this,
          SLOT(generate_process_path_handler()));
  connect(this, SIGNAL(selection_changed()), this, SLOT(selection_changed_handler()));
  connect(this, SIGNAL(surface_detection_started()), this,
          SLOT(surface_detection_started_handler()));
  connect(this, SIGNAL(surface_detection_completed()), this,
          SLOT(surface_detection_completed_handler()));
  connect(this, SIGNAL(connect_started()), this, SLOT(connect_started_handler()));
  connect(this, SIGNAL(connect_completed()), this, SLOT(connect_completed_handler()));

  connect(ui_.pushButtonSavePlan, SIGNAL(clicked(bool)), this, SLOT(save_motion_plan_handler()));
  connect(ui_.PushButtonOpenFile, SIGNAL(clicked(bool)), this, SLOT(load_motion_plan_handler()));

  connect(ui_.pushButtonExecutePath, SIGNAL(clicked()), this, SLOT(execute_motion_plan_handler()));
  connect(ui_.pushButtonSimulatePath, SIGNAL(clicked(bool)), this,
          SLOT(simulate_motion_plan_handler()));

  connect(robot_scan_config_window_, SIGNAL(parameters_save_requested()), this,
          SLOT(request_save_parameters()));
  connect(surface_detect_config_window_, SIGNAL(parameters_save_requested()), this,
          SLOT(request_save_parameters()));
  connect(robot_blend_config_window_, SIGNAL(parameters_save_requested()), this,
          SLOT(request_save_parameters()));
  connect(ui_.ListWidgetSelectedSurfs, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this,
          SLOT(handle_surface_rename(QListWidgetItem*)));
  connect(scan_plan_config_window_, SIGNAL(parameters_save_requested()), this,
          SLOT(request_save_parameters()));

  // moving to first tab
  ui_.TabWidgetCreateLib->setCurrentIndex(0);

  // setting up timer
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_handler()));
  timer->start(4000);

  // connect from a separate thread
  QFuture<void> future = QtConcurrent::run(this, &RobotBlendingWidget::connect_to_services);
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
  while (ros::ok())
  {
    ROS_INFO_STREAM("rviz blending panel connecting to services");
    if (surface_detection_client_.waitForExistence(ros::Duration(2)) &&
        select_surface_client_.waitForExistence(ros::Duration(2)) &&
        surface_blending_parameters_client_.waitForExistence(ros::Duration(2)))
    {

      ROS_INFO_STREAM("rviz panel connected to the services '"
                      << surface_detection_client_.getService() << "' and '"
                      << select_surface_client_.getService() << "'");

      // requesting parameters
      if (surface_blending_parameters_client_.call(req, res))
      {
        robot_scan_config_window_->params() = res.robot_scan;
        surface_detect_config_window_->params() = res.surface_detection;
        robot_scan_parameters_ = res.robot_scan;
        surf_detect_parameters_ = res.surface_detection;
        blending_plan_parameters_ = res.blending_plan;
        robot_blend_config_window_->params() = res.blending_plan;
        scan_plan_config_window_->params() = res.scan_plan;

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
      ROS_ERROR_STREAM("rviz panel could not connect to one or more ros services:\n\t'"
                       << surface_detection_client_.getService() << "'\n\t'"
                       << select_surface_client_.getService() << "'\n\t'"
                       << surface_blending_parameters_client_.getService());
    }
  }
}

bool RobotBlendingWidget::call_select_surface_service(godel_msgs::SelectSurface::Request& req)
{
  godel_msgs::SelectSurface::Response res;
  bool succeeded = select_surface_client_.call(req, res);
  if (succeeded)
  {
  }
  else
  {
  }

  return succeeded;
}

bool RobotBlendingWidget::call_surface_detection_service(godel_msgs::SurfaceDetection& s)
{
  bool succeeded = surface_detection_client_.call(s.request, s.response);
  if (succeeded)
  {
  }
  else
  {
  }

  return succeeded;
}

void RobotBlendingWidget::selected_surface_changed_callback(
    godel_msgs::SelectedSurfacesChangedConstPtr msg)
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
  robot_scan_parameters_ = robot_scan_config_window_->params();

  // updating entries in gui
  ui_.LineEditSensorTopic->setText(QString::fromStdString(robot_scan_parameters_.scan_topic));
  ui_.SpinBoxNumScans->setValue(static_cast<int>(robot_scan_parameters_.num_scan_points));
  ui_.LineEditCamTilt->setText(
      QString::number(RAD_TO_DEGREES * robot_scan_parameters_.cam_tilt_angle));
  ui_.LineEditSweepAngleStart->setText(
      QString::number(RAD_TO_DEGREES * robot_scan_parameters_.sweep_angle_start));
  ui_.LineEditSweepAngleEnd->setText(
      QString::number(RAD_TO_DEGREES * robot_scan_parameters_.sweep_angle_end));

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
  surf_detect_parameters_ = surface_detect_config_window_->params();
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
  robot_scan_config_window_->params() = robot_scan_parameters_;
  robot_scan_config_window_->show();
}

void RobotBlendingWidget::blend_options_click_handler() { robot_blend_config_window_->show(); }

void RobotBlendingWidget::scan_plan_options_click_handler() { scan_plan_config_window_->show(); }

void RobotBlendingWidget::surface_options_click_handler()
{
  surface_detect_config_window_->params() = surf_detect_parameters_;
  surface_detect_config_window_->show();
}

void RobotBlendingWidget::update_handler() {}

void RobotBlendingWidget::selection_changed_handler()
{
  std::vector<std::string> list = selected_surfaces_msg_.selected_surfaces;

  ui_.ListWidgetSelectedSurfs->clear();
  if (list.size() > 0)
  {
    for (std::vector<std::string>::iterator i = list.begin(); i != list.end(); i++)
    {
      QListWidgetItem* item = new QListWidgetItem();
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
  QFuture<void> future = QtConcurrent::run(this, &RobotBlendingWidget::send_scan_and_find_request);
}

void RobotBlendingWidget::find_surface_button_handler()
{
  QFuture<void> future = QtConcurrent::run(this, &RobotBlendingWidget::send_find_surface_request);
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

  if (call_surface_detection_service(s))
  {
    if (s.response.surfaces_found)
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

  if (call_surface_detection_service(s))
  {
    if (s.response.surfaces_found)
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
  if (surface_detection_op_succeeded_)
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
  process_plan.request.params = robot_blend_config_window_->params();
  process_plan.request.scan_params = this->scan_plan_config_window_->params();
  process_plan.request.action = process_plan.request.GENERATE_MOTION_PLAN_AND_PREVIEW;
  ROS_INFO_STREAM("process plan request sent");
  if (process_plan_client_.call(process_plan))
  {
    std::vector<std::string> plan_names;
    request_available_motions(plan_names);
    update_motion_plan_list(plan_names);

    ui_.TabWidgetCreateLib->setCurrentIndex(2);
  }
}

void RobotBlendingWidget::update_motion_plan_list(const std::vector<std::string>& names)
{
  ui_.ListPathResults->clear();
  for (std::size_t i = 0; i < names.size(); ++i)
  {
    QListWidgetItem* item = new QListWidgetItem();
    item->setText(QString::fromStdString(names[i]));
    ui_.ListPathResults->addItem(item);
  }
}

void RobotBlendingWidget::save_robot_scan_parameters()
{
  robot_scan_parameters_.num_scan_points = ui_.SpinBoxNumScans->value();
  robot_scan_parameters_.cam_tilt_angle = ui_.LineEditCamTilt->text().toDouble() * DEGREES_TO_RAD;
  robot_scan_parameters_.sweep_angle_start =
      ui_.LineEditSweepAngleStart->text().toDouble() * DEGREES_TO_RAD;
  robot_scan_parameters_.sweep_angle_end =
      ui_.LineEditSweepAngleEnd->text().toDouble() * DEGREES_TO_RAD;
  robot_scan_parameters_.scan_topic = ui_.LineEditSensorTopic->text().toStdString();
}

void RobotBlendingWidget::request_available_motions(std::vector<std::string>& plans)
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

void RobotBlendingWidget::select_motion_plan(const std::string& name, bool simulate)
{
  godel_msgs::SelectMotionPlan srv;
  srv.request.name = name;
  srv.request.simulate = simulate;
  select_motion_plan_client_.call(srv);
}

void RobotBlendingWidget::simulate_motion_plan_handler()
{
  if (ui_.ListPathResults->currentItem() == NULL)
    return;
  std::string name = ui_.ListPathResults->currentItem()->text().toStdString();
  ROS_INFO_STREAM("Selected " << name << " to be simulated");
  if (!name.empty())
    select_motion_plan(name, true);
}

void RobotBlendingWidget::execute_motion_plan_handler()
{
  if (ui_.ListPathResults->currentItem() == NULL)
    return;
  std::string name = ui_.ListPathResults->currentItem()->text().toStdString();
  ROS_INFO_STREAM("Selected " << name << " to be executed");
  if (!name.empty())
    select_motion_plan(name, false);
}

void RobotBlendingWidget::save_motion_plan_handler()
{
  QString filepath = QFileDialog::getSaveFileName(this, "Save Motion Plan");
  std::string path = filepath.toStdString();
  if (!path.empty())
  {
    ROS_DEBUG_STREAM("You want to save motion plan to: " << path);
    request_load_save_motions(path, false);
  }
}

void RobotBlendingWidget::load_motion_plan_handler()
{
  QString filepath = QFileDialog::getOpenFileName(this, "Load Motion Plan");
  std::string path = filepath.toStdString();
  if (!path.empty())
  {
    ROS_DEBUG_STREAM("You want to load a motion plan from: " << path);
    request_load_save_motions(path, true);
  }
}

void RobotBlendingWidget::request_load_save_motions(const std::string& path, bool isLoad)
{
  // Pre-condition: The path must not be an empty string
  if (path.empty())
  {
    ROS_WARN("Cannot save or load an empty path");
    return;
  }

  godel_msgs::LoadSaveMotionPlan srv;
  srv.request.path = path;

  if (isLoad)
  {
    srv.request.mode = godel_msgs::LoadSaveMotionPlan::Request::MODE_LOAD;
  }
  else
  {
    srv.request.mode = godel_msgs::LoadSaveMotionPlan::Request::MODE_SAVE;
  }

  if (load_save_motion_plan_client_.call(srv))
  {
    std::vector<std::string> plans;
    request_available_motions(plans);
    update_motion_plan_list(plans);
  }
  else
  {
    ROS_WARN_STREAM("Blending service unable to " << (isLoad ? "load" : "save")
                                                  << "plan: " << path);
  }
}

void RobotBlendingWidget::request_save_parameters()
{
  godel_msgs::SurfaceBlendingParameters::Request req;
  req.action = godel_msgs::SurfaceBlendingParameters::Request::SAVE_PARAMETERS;
  req.surface_detection = surface_detect_config_window_->params();
  req.blending_plan = robot_blend_config_window_->params();
  req.robot_scan = robot_scan_config_window_->params();
  req.scan_plan = scan_plan_config_window_->params();

  godel_msgs::SurfaceBlendingParameters::Response res;
  if (!surface_blending_parameters_client_.call(req, res))
  {
    ROS_WARN_STREAM("Could not complete service call to save your parameters!");
  }
}

void RobotBlendingWidget::handle_surface_rename(QListWidgetItem* item)
{
  if (!item)
    return;

  QString old_text = item->text();
  // spawn window to prompt a rename
  QString new_text = QInputDialog::getText(this, "Surface Rename", "Enter a new surface name: ");

  if (!new_text.isEmpty())
  {
    godel_msgs::RenameSurfaceResponse res;
    godel_msgs::RenameSurfaceRequest req;
    req.old_name = old_text.toStdString();
    req.new_name = new_text.toStdString();
    if (rename_surface_client_.call(req, res))
    {
      item->setText(new_text);
    }
    else
    {
      ROS_WARN_STREAM("Failed to update the name of surface " << old_text.toStdString());
    }
  }
}

} /* namespace widgets */
}
