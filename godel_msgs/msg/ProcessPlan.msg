# Godel supports two processes at the moment: surface blending and line scanning
# This msg encapsulates a motion plan for one of these components.

# Enumeration represeting plan type  
int32 BLEND_TYPE=0
int32 SCAN_TYPE=1

# The plan is broken into components so that nodes responsible
# for the actual execution can perform any necessary IO commands
# as the robot begins and completes the actual process path. In
# our case, the blending tool and laser scanner turn on after the 
# approach and off after the process path is done.
trajectory_msgs/JointTrajectory trajectory_approach
trajectory_msgs/JointTrajectory trajectory_process
trajectory_msgs/JointTrajectory trajectory_depart
int32 type
