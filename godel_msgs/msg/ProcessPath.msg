# ProcessPath

# The frame of reference for poses is assumed to be the root of the
# workspace as specified in the URDF

# The process path consists of some number of 'pose sequences' each of which
# represents a segment of the path in which the tool constraints must be held.
# In between segments, the tool can 'retract' and the robot can change config.
geometry_msgs/PoseArray[] segments
