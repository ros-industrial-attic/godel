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
#include <actionlib/client/simple_action_client.h>
#include <godel_msgs/ProcessPlanningAction.h>

const double RAD_TO_DEGREES = 180.0f / M_PI;
const double DEGREES_TO_RAD = M_PI / 180.0f;
const static std::string PROCESS_PLANNING_ACTION_SERVER_NAME = "process_planning_as";
const static std::string SELECT_MOTION_PLAN_ACTION_SERVER_NAME = "select_motion_plan_as";

namespace godel_plugins
{
namespace widgets
{

RobotBlendingWidget::RobotBlendingWidget(std::string ns) :
  param_ns_(ns),
  process_planning_action_client_(PROCESS_PLANNING_ACTION_SERVER_NAME, true),
  select_motion_plan_action_client_(SELECT_MOTION_PLAN_ACTION_SERVER_NAME, true)
{
  init();
}

RobotBlendingWidget::~RobotBlendingWidget()
{
  delete robot_scan_config_window_;
  delete surface_detect_config_window_;
  delete path_planning_config_window_;
  delete scan_plan_config_window_;
}

void RobotBlendingWidget::init()
{

  // initializing ros comm interface
  ros::NodeHandle nh("");
  surface_detection_client_ = nh.serviceClient<godel_msgs::SurfaceDetection>(SURFACE_DETECTION_SERVICE);
  select_surface_client_ = nh.serviceClient<godel_msgs::SelectSurface>(SELECT_SURFACE_SERVICE);
  get_motion_plans_client_ = nh.serviceClient<godel_msgs::GetAvailableMotionPlans>(GET_AVAILABLE_MOTION_PLANS_SERVICE);
  select_motion_plan_client_ = nh.serviceClient<godel_msgs::SelectMotionPlan>(SELECT_MOTION_PLAN_SERVICE);
  load_save_motion_plan_client_ = nh.serviceClient<godel_msgs::LoadSaveMotionPlan>(LOAD_SAVE_MOTION_PLAN_SERVICE);
  rename_surface_client_ = nh.serviceClient<godel_msgs::RenameSurface>(RENAME_SURFACE_SERVICE);

  surface_blending_parameters_client_ =
      nh.serviceClient<godel_msgs::SurfaceBlendingParameters>(SURFACE_BLENDING_PARAMETERS_SERVICE);

  selected_surfaces_subs_ =
      nh.subscribe(SELECTED_SURFACES_CHANGED_TOPIC, 1, &RobotBlendingWidget::selected_surface_changed_callback, this);

  // Initialize UI
  ui_.setupUi(this);

  // Initialize config windows
  robot_scan_config_window_ = new RobotScanConfigWidget(robot_scan_parameters_);
  surface_detect_config_window_ = new SurfaceDetectionConfigWidget(surf_detect_parameters_);
  path_planning_config_window_ = new PathPlanningConfigWidget(path_planning_parameters_);
  scan_plan_config_window_ = new ScanPlanConfigWidget(scan_plan_parameters_);


  /* Setting signals and slots */

  // Options Handlers
  connect(ui_.PushButtonPathPlanningOptions, SIGNAL(clicked()), this, SLOT(path_planning_options_click_handler()));
  connect(ui_.pushButtonProfileOptions, SIGNAL(clicked()), this, SLOT(scan_plan_options_click_handler()));

  // General Navigation Buttons
  connect(ui_.PushButtonNext, SIGNAL(clicked()), this, SLOT(increase_tab_index_handler()));
  connect(ui_.PushButtonBack, SIGNAL(clicked()), this, SLOT(decrease_tab_index_handler()));

  // Scan and Find Buttons
  connect(ui_.PushButtonMoreOptions, SIGNAL(clicked()), this, SLOT(scan_options_click_handler()));
  connect(ui_.PushButtonSurfaceOptions, SIGNAL(clicked()), this,
          SLOT(surface_options_click_handler()));
  connect(ui_.PushButtonPreviewPath, SIGNAL(clicked()), this, SLOT(preview_path_handler()));
  connect(ui_.PushButtonScan, SIGNAL(clicked()), this, SLOT(scan_button_handler()));
  connect(ui_.PushButtonFindSurface, SIGNAL(clicked()), this, SLOT(find_surface_button_handler()));

  // Select Surface Buttons
  connect(ui_.PushButtonSelectAllSurfaces, SIGNAL(clicked()), this, SLOT(select_all_handler()));
  connect(ui_.PushButtonDeselectAllSurfaces, SIGNAL(clicked()), this, SLOT(deselect_all_handler()));
  connect(ui_.PushButtonHideAllSurfaces, SIGNAL(clicked()), this, SLOT(hide_all_handler()));
  connect(ui_.PushButtonShowAllSurfaces, SIGNAL(clicked()), this, SLOT(show_all_handler()));

  // Path Generation
  connect(ui_.PushButtonGeneratePaths, SIGNAL(clicked()), this, SLOT(generate_process_path_handler()));

  // Path Simulation, Execution, and Saving
  connect(ui_.pushButtonExecutePath, SIGNAL(clicked()), this, SLOT(execute_motion_plan_handler()));
  connect(ui_.pushButtonSimulatePath, SIGNAL(clicked(bool)), this, SLOT(simulate_motion_plan_handler()));
  connect(ui_.pushButtonSavePlan, SIGNAL(clicked(bool)), this, SLOT(save_motion_plan_handler()));
  connect(ui_.PushButtonOpenFile, SIGNAL(clicked(bool)), this, SLOT(load_motion_plan_handler()));

  // Intermediate Signals (Generated by other UI actions)
  connect(this, SIGNAL(selection_changed()), this, SLOT(selected_surfaces_changed_handler()));
  connect(this, SIGNAL(surface_detection_started()), this, SLOT(surface_detection_started_handler()));
  connect(this, SIGNAL(surface_detection_completed()), this, SLOT(surface_detection_completed_handler()));
  connect(this, SIGNAL(connect_started()), this, SLOT(connect_started_handler()));
  connect(this, SIGNAL(connect_completed()), this, SLOT(connect_completed_handler()));


  // Configuration Windows
  connect(surface_detect_config_window_, SIGNAL(parameters_save_requested()), this, SLOT(request_save_parameters()));
  connect(robot_scan_config_window_, SIGNAL(parameters_changed()), this, SLOT(robot_scan_params_changed_handler()));
  connect(robot_scan_config_window_, SIGNAL(parameters_save_requested()), this, SLOT(request_save_parameters()));
  connect(path_planning_config_window_, SIGNAL(parameters_save_requested()), this, SLOT(request_save_parameters()));
  connect(scan_plan_config_window_, SIGNAL(parameters_save_requested()), this, SLOT(request_save_parameters()));
  connect(surface_detect_config_window_, SIGNAL(parameters_changed()), this,
          SLOT(surface_detect_params_changed_handler()));

  // Feedback updates
  connect(ui_.ListWidgetSelectedSurfs, SIGNAL(itemDoubleClicked(QListWidgetItem*)), this,
          SLOT(handle_surface_rename(QListWidgetItem*)));
  connect(this, SIGNAL(feedbackReceived(QString)), this, SLOT(setFeedbackText(QString)));


  // Move to first tab
  ui_.TabWidgetCreateLib->setCurrentIndex(0);
  ROS_INFO_STREAM("Current Index " << ui_.TabWidgetCreateLib->currentIndex());
  
  // Setup timer
  QTimer* timer = new QTimer(this);
  connect(timer, SIGNAL(timeout()), this, SLOT(update_handler()));
  timer->start(4000);

  // Connect to services from a separate thread
  QtConcurrent::run(this, &RobotBlendingWidget::connect_to_services);
}


void RobotBlendingWidget::connect_to_services()
{
  // call services to get parameters
  godel_msgs::SurfaceBlendingParameters msg;
  msg.request.action = msg.request.GET_CURRENT_PARAMETERS;

  // disable gui interaction
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
      if (surface_blending_parameters_client_.call(msg.request, msg.response))
      {
        robot_scan_config_window_->params() = msg.response.robot_scan;
        surface_detect_config_window_->params() = msg.response.surface_detection;
        robot_scan_parameters_ = msg.response.robot_scan;
        surf_detect_parameters_ = msg.response.surface_detection;
        //blending_plan_parameters_ = msg.response.blending_plan;
        path_planning_parameters_ = msg.response.path_params;
        path_planning_config_window_->params() = msg.response.path_params;
        scan_plan_config_window_->params() = msg.response.scan_plan;

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


void RobotBlendingWidget::selected_surface_changed_callback(
    godel_msgs::SelectedSurfacesChangedConstPtr msg)
{
  selected_surfaces_msg_ = *msg;
  emit_signal_selection_change();
}


void RobotBlendingWidget::select_all_handler()
{
  godel_msgs::SelectSurface msg;
  msg.request.action = msg.request.SELECT_ALL;
  select_surface_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::deselect_all_handler()
{
  godel_msgs::SelectSurface msg;
  msg.request.action = msg.request.DESELECT_ALL;
  select_surface_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::hide_all_handler()
{
  godel_msgs::SelectSurface msg;
  msg.request.action = msg.request.HIDE_ALL;
  select_surface_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::show_all_handler()
{
  godel_msgs::SelectSurface msg;
  msg.request.action = msg.request.SHOW_ALL;
  select_surface_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::robot_scan_params_changed_handler()
{
  robot_scan_parameters_ = robot_scan_config_window_->params();

  // updating entries in gui
  ui_.LineEditSensorTopic->setText(QString::fromStdString(robot_scan_parameters_.scan_topic));
  ui_.SpinBoxNumScans->setValue(static_cast<int>(robot_scan_parameters_.num_scan_points));
  ui_.LineEditCamTilt->setText(QString::number(RAD_TO_DEGREES * robot_scan_parameters_.cam_tilt_angle));
  ui_.LineEditSweepAngleStart->setText(QString::number(RAD_TO_DEGREES * robot_scan_parameters_.sweep_angle_start));
  ui_.LineEditSweepAngleEnd->setText(QString::number(RAD_TO_DEGREES * robot_scan_parameters_.sweep_angle_end));

  // request publish scan path
  godel_msgs::SurfaceDetection msg;
  msg.request.action = msg.request.PUBLISH_SCAN_PATH;
  msg.request.use_default_parameters = false;
  msg.request.robot_scan = robot_scan_parameters_;
  msg.request.surface_detection = surf_detect_parameters_;
  surface_detection_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::surface_detect_params_changed_handler()
{
  surf_detect_parameters_ = surface_detect_config_window_->params();
}


void RobotBlendingWidget::preview_path_handler()
{
  save_robot_scan_parameters();

  // request publish scan path
  godel_msgs::SurfaceDetection msg;
  msg.request.action = msg.request.PUBLISH_SCAN_PATH;
  msg.request.use_default_parameters = false;
  msg.request.robot_scan = robot_scan_parameters_;
  msg.request.surface_detection = surf_detect_parameters_;
  surface_detection_client_.call(msg.request, msg.response);
}


void RobotBlendingWidget::scan_options_click_handler()
{
  save_robot_scan_parameters();
  robot_scan_config_window_->params() = robot_scan_parameters_;
  robot_scan_config_window_->show();
}

void RobotBlendingWidget::path_planning_options_click_handler()
{
  path_planning_config_window_->show();
}


void RobotBlendingWidget::scan_plan_options_click_handler()
{
  scan_plan_config_window_->show();
}


void RobotBlendingWidget::surface_options_click_handler()
{
  surface_detect_config_window_->params() = surf_detect_parameters_;
  surface_detect_config_window_->show();
}


void RobotBlendingWidget::selected_surfaces_changed_handler()
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
    if (ui_.TabWidgetCreateLib->currentIndex() < 2)
        ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() + 1);
}


void RobotBlendingWidget::decrease_tab_index_handler()
{
    if (ui_.TabWidgetCreateLib->currentIndex() > 0)
        ui_.TabWidgetCreateLib->setCurrentIndex(ui_.TabWidgetCreateLib->currentIndex() - 1);
}


void RobotBlendingWidget::scan_button_handler()
{
  QtConcurrent::run(this, &RobotBlendingWidget::send_scan_and_find_request);
}


void RobotBlendingWidget::find_surface_button_handler()
{
  QtConcurrent::run(this, &RobotBlendingWidget::send_find_surface_request);
}


void RobotBlendingWidget::send_find_surface_request()
{
  surface_detection_op_message_ = "FIND IN PROGRESS";

  // disable gui
  Q_EMIT surface_detection_started();

  // creating surface detection request
  godel_msgs::SurfaceDetection msg;
  msg.request.action = msg.request.FIND_ONLY;
  msg.request.use_default_parameters = false;
  msg.request.robot_scan = robot_scan_parameters_;
  msg.request.surface_detection = surf_detect_parameters_;

  if (surface_detection_client_.call(msg.request, msg.response))
  {
    if (msg.response.surfaces_found)
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
  godel_msgs::SurfaceDetection msg;
  msg.request.action = msg.request.SCAN_AND_FIND_ONLY;
  msg.request.use_default_parameters = false;
  msg.request.robot_scan = robot_scan_parameters_;
  msg.request.surface_detection = surf_detect_parameters_;

  if (surface_detection_client_.call(msg.request, msg.response))
  {
    if (msg.response.surfaces_found)
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
    ui_.TabWidgetCreateLib->setCurrentIndex(1);

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
  ui_.textEditFeedback->setText(QString::fromStdString("Sending planning request"));
  ui_.PushButtonGeneratePaths->setEnabled(false);
  process_planning_action_client_.waitForServer();

  godel_msgs::ProcessPlanningActionGoal goal;
  goal.goal.use_default_parameters = false;
  goal.goal.params = path_planning_config_window_->params();
  goal.goal.action = goal.goal.GENERATE_MOTION_PLAN_AND_PREVIEW;
  process_planning_action_client_.sendGoal(
        goal.goal,
        boost::bind(&RobotBlendingWidget::processPlanningDoneCallback, this, _1, _2),
        boost::bind(&RobotBlendingWidget::processPlanningActiveCallback, this),
        boost::bind(&RobotBlendingWidget::processPlanningFeedbackCallback, this, _1)
        );
}

void RobotBlendingWidget::processPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
            const godel_msgs::ProcessPlanningResultConstPtr& result)
{
  if(result->succeeded)
  {
    std::vector<std::string> plan_names;
    request_available_motions(plan_names);
    update_motion_plan_list(plan_names);
    ui_.PushButtonGeneratePaths->setEnabled(true);
    ui_.TabWidgetCreateLib->setCurrentIndex(2);
  }
  else
    ROS_WARN_STREAM("Process Planning action failed");
}

// Called once when the goal becomes active
void RobotBlendingWidget::processPlanningActiveCallback()
{
  ROS_INFO("Process Planning goal just went active");
}

void RobotBlendingWidget::processPlanningFeedbackCallback(
    const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback)
{  
  Q_EMIT feedbackReceived(QString::fromStdString((feedback->last_completed).c_str()));
}


void RobotBlendingWidget::setFeedbackText(QString feedback)
{
  ui_.textEditFeedback->moveCursor(QTextCursor::End);
  ui_.textEditFeedback->insertPlainText(QString::fromStdString("\n").append(feedback));
  ui_.textEditFeedback->moveCursor(QTextCursor::End);
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
  ROS_INFO_STREAM("In select motion plan");
  godel_msgs::SelectMotionPlanActionGoal goal;
  goal.goal.name = name;
  goal.goal.simulate = simulate;
  goal.goal.wait_for_execution = true;
  select_motion_plan_action_client_.sendGoal(goal.goal);
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
  req.path_params = path_planning_config_window_->params();
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


void RobotBlendingWidget::on_ListPathResults_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous)
{
  if(current != NULL)
  {
    QString new_selection = current->text();
    godel_msgs::SurfaceDetectionRequest req;
    req.action = godel_msgs::SurfaceDetection::Request::VISUALIZATION_REQUEST;
    req.name = new_selection.toStdString();
    godel_msgs::SurfaceDetection::Response res;
    if(!surface_detection_client_.call(req,res))
      ROS_WARN_STREAM("Could not complete service call to visualize: " << req.name);
  }
}


} /* namespace widgets */
}


