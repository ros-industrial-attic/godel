#ifndef ROBOT_SCAN_PARAMS_HELPER_H
#define ROBOT_SCAN_PARAMS_HELPER_H

#include <param_helpers/param_interfaces.h>
#include <godel_msgs/RobotScanParameters.h>

namespace param_helpers 
{

template<>
struct ParamInterfaces<godel_msgs::RobotScanParameters>
{
  template<typename Setter>
  static void save(Setter& setter, const godel_msgs::RobotScanParameters& params)
  {
    setter.set("group_name", params.group_name);
    setter.set("home_position", params.home_position);
    setter.set("world_frame", params.world_frame);
    setter.set("tcp_frame", params.tcp_frame);

    setter.set("cam_to_obj_zoffset", params.cam_to_obj_zoffset);
    setter.set("cam_to_obj_xoffset", params.cam_to_obj_xoffset);
    setter.set("cam_tilt_angle", params.cam_tilt_angle);

    setter.set("sweep_angle_start", params.sweep_angle_start);
    setter.set("sweep_angle_end", params.sweep_angle_end);

    setter.set("num_scan_points", params.num_scan_points);
    setter.set("reachable_scan_points_ratio", params.reachable_scan_points_ratio);
    setter.set("stop_on_planning_error", params.stop_on_planning_error);

    setter.set("scan_topic", params.scan_topic);
    setter.set("scan_target_frame", params.scan_target_frame);

    // Now we must generate the more complicated pose objects
    setter.set("tcp_to_cam_pose/trans/x", params.tcp_to_cam_pose.position.x);
    setter.set("tcp_to_cam_pose/trans/y", params.tcp_to_cam_pose.position.y);
    setter.set("tcp_to_cam_pose/trans/z", params.tcp_to_cam_pose.position.z);
    
    setter.set("tcp_to_cam_pose/quat/x", params.tcp_to_cam_pose.orientation.x);
    setter.set("tcp_to_cam_pose/quat/y", params.tcp_to_cam_pose.orientation.y);
    setter.set("tcp_to_cam_pose/quat/z", params.tcp_to_cam_pose.orientation.z);
    setter.set("tcp_to_cam_pose/quat/w", params.tcp_to_cam_pose.orientation.w);
  }

  template<typename Getter>
  static void load(Getter& getter, godel_msgs::RobotScanParameters& params)
  {
    getter.get("group_name", params.group_name);
    getter.get("home_position", params.home_position);
    getter.get("world_frame", params.world_frame);
    getter.get("tcp_frame", params.tcp_frame);

    getter.get("cam_to_obj_zoffset", params.cam_to_obj_zoffset);
    getter.get("cam_to_obj_xoffset", params.cam_to_obj_xoffset);
    getter.get("cam_tilt_angle", params.cam_tilt_angle);

    getter.get("sweep_angle_start", params.sweep_angle_start);
    getter.get("sweep_angle_end", params.sweep_angle_end);

    getter.get("num_scan_points", params.num_scan_points);
    getter.get("reachable_scan_points_ratio", params.reachable_scan_points_ratio);
    getter.get("stop_on_planning_error", params.stop_on_planning_error);

    getter.get("scan_topic", params.scan_topic);
    getter.get("scan_target_frame", params.scan_target_frame);

    // Now we must generate the more complicated pose objects
    getter.get("tcp_to_cam_pose/trans/x", params.tcp_to_cam_pose.position.x);
    getter.get("tcp_to_cam_pose/trans/y", params.tcp_to_cam_pose.position.y);
    getter.get("tcp_to_cam_pose/trans/z", params.tcp_to_cam_pose.position.z);
    
    getter.get("tcp_to_cam_pose/quat/x", params.tcp_to_cam_pose.orientation.x);
    getter.get("tcp_to_cam_pose/quat/y", params.tcp_to_cam_pose.orientation.y);
    getter.get("tcp_to_cam_pose/quat/z", params.tcp_to_cam_pose.orientation.z);
    getter.get("tcp_to_cam_pose/quat/w", params.tcp_to_cam_pose.orientation.w);
  }
};

}

#endif
