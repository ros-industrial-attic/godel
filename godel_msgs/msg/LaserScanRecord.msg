# This msg defines a data structure that encodes the raw-data results a QA process execution using a line scanner
# Specifically, it records the laser scans and the estimated tcp for any data taken during the execution.

std_msgs/Header header
sensor_msgs/PointCloud2[] profiles
geometry_msgs/Pose[] poses
