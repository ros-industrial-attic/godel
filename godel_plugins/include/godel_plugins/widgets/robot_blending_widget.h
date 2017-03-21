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
#include <godel_msgs/SurfaceBlendingParameters.h>
#include <godel_msgs/BlendingPlanParameters.h>
#include <godel_msgs/ScanPlanParameters.h>
#include <godel_msgs/ProcessPlanningAction.h>
#include <godel_msgs/SelectMotionPlanAction.h>

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

#include <ui_robot_blending_plugin.h>

#include <QWidget>
#include <QTimer>
#include <QtConcurrent/QtConcurrentRun>

#include <godel_plugins/widgets/surface_detection_configuration.h>
#include <godel_plugins/widgets/robot_scan_configuration.h>
#include <godel_plugins/widgets/path_planning_param_window.h>
#include <godel_plugins/widgets/scan_tool_configuration_window.h>

// macros
#ifndef DEG2RAD
#define DEG2RAD(x) ((x)*0.017453293)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x) ((x)*57.29578)
#endif

namespace godel_plugins
{
namespace widgets
{

const std::string SURFACE_DETECTION_SERVICE = "surface_detection";
const std::string SURFACE_BLENDING_PARAMETERS_SERVICE = "surface_blending_parameters";
const std::string SELECT_SURFACE_SERVICE = "select_surface";
const std::string PROCESS_PATH_SERVICE = "process_path";
const std::string SELECTED_SURFACES_CHANGED_TOPIC = "selected_surfaces_changed";
const std::string GET_AVAILABLE_MOTION_PLANS_SERVICE = "get_available_motion_plans";
const std::string SELECT_MOTION_PLAN_SERVICE = "select_motion_plan";
const std::string LOAD_SAVE_MOTION_PLAN_SERVICE = "load_save_motion_plan";
const std::string RENAME_SURFACE_SERVICE = "rename_surface";

class RobotBlendingWidget : public QWidget
{
  Q_OBJECT
public:
  RobotBlendingWidget(std::string ns = "");
  virtual ~RobotBlendingWidget();

  std::string get_name() { return "RobotBlending"; }

  int width() { return ui_.TabWidget->width(); }

  int height() { return ui_.TabWidget->height(); }

  void emit_signal_selection_change() { Q_EMIT selection_changed(); }

Q_SIGNALS:
  void selection_changed();
  void surface_detection_started();
  void surface_detection_completed();
  void connect_started();
  void connect_completed();
  void feedbackReceived(QString feedback);

protected:
  void init();
  void send_scan_and_find_request();
  void send_find_surface_request();
  void save_robot_scan_parameters();
  bool call_select_surface_service(godel_msgs::SelectSurface::Request& req);
  bool call_surface_detection_service(godel_msgs::SurfaceDetection& s);
  void selected_surface_changed_callback(godel_msgs::SelectedSurfacesChangedConstPtr msg);
  void select_motion_plan(const std::string& name, bool simulate);
  void request_available_motions(std::vector<std::string>& plans);
  void request_load_save_motions(const std::string& path, bool isLoad);
  void update_motion_plan_list(const std::vector<std::string>& names);

protected Q_SLOTS:

  void scan_button_handler();
  void find_surface_button_handler();
  void connect_to_services();
  void increase_tab_index_handler();
  void decrease_tab_index_handler();
  void selected_surfaces_changed_handler();
  void select_all_handler();
  void deselect_all_handler();
  void hide_all_handler();
  void show_all_handler();
  void scan_options_click_handler();

  void path_planning_options_click_handler();
  void scan_plan_options_click_handler();

  void surface_options_click_handler();
  void robot_scan_params_changed_handler();
  void surface_detect_params_changed_handler();
  void preview_path_handler();
  void surface_detection_started_handler();
  void surface_detection_completed_handler();
  void connect_started_handler();
  void connect_completed_handler();
  void generate_process_path_handler();
  void request_save_parameters();

  void simulate_motion_plan_handler();
  void execute_motion_plan_handler();

  void save_motion_plan_handler();
  void load_motion_plan_handler();

  void handle_surface_rename(QListWidgetItem* item);
  void on_ListPathResults_currentItemChanged(QListWidgetItem *current, QListWidgetItem *previous);

  void setFeedbackText(QString feedback);

  // Action Client Callbacks
  void processPlanningFeedbackCallback(const godel_msgs::ProcessPlanningFeedbackConstPtr& feedback);
  void processPlanningDoneCallback(const actionlib::SimpleClientGoalState& state,
                                   const godel_msgs::ProcessPlanningResultConstPtr& result);
  void processPlanningActiveCallback();


protected:
  Ui::RobotBlendingWidget ui_;
  RobotScanConfigWidget* robot_scan_config_window_;
  PathPlanningConfigWidget* path_planning_config_window_;
  SurfaceDetectionConfigWidget* surface_detect_config_window_;
  ScanPlanConfigWidget* scan_plan_config_window_;

  ros::ServiceClient surface_detection_client_;
  ros::ServiceClient select_surface_client_;
  ros::ServiceClient process_plan_client_;
  ros::ServiceClient surface_blending_parameters_client_;
  ros::ServiceClient get_motion_plans_client_;
  ros::ServiceClient select_motion_plan_client_;
  ros::ServiceClient load_save_motion_plan_client_;
  ros::ServiceClient rename_surface_client_;
  ros::Subscriber selected_surfaces_subs_;
  actionlib::SimpleActionClient<godel_msgs::ProcessPlanningAction> process_planning_action_client_;
  actionlib::SimpleActionClient<godel_msgs::SelectMotionPlanAction> select_motion_plan_action_client_;

  std::string param_ns_;
  godel_msgs::RobotScanParameters robot_scan_parameters_;
  godel_msgs::SurfaceDetectionParameters surf_detect_parameters_;
  godel_msgs::PathPlanningParameters path_planning_parameters_;
  godel_msgs::ScanPlanParameters scan_plan_parameters_;
  godel_msgs::SurfaceDetection::Response latest_result_;
  godel_msgs::SurfaceDetection::Request latest_request_;
  godel_msgs::SelectedSurfacesChanged selected_surfaces_msg_;

  // service results
  bool surface_detection_op_succeeded_;
  std::string surface_detection_op_message_;
};

} /* namespace widgets */
}
#endif /* ROBOT_BLENDING_WIDGET_H_ */
