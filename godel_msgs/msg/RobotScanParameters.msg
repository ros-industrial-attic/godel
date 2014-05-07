# path planning
string group_name
string world_frame
string tcp_frame

# trajectory generation
geometry_msgs/Pose tcp_to_cam_pose
geometry_msgs/Pose world_to_obj_pose
float64 cam_to_obj_zoffset
float64 cam_to_obj_xoffset
float64 cam_tilt_angle # rotation about relative object's y axis (radians)
float64 sweep_angle_start
float64 sweep_angle_end
int32 num_scan_point32s
float64 reachable_scan_point32s_ratio
bool stop_on_planning_error

# scan data source
string scan_topic
string scan_target_frame
