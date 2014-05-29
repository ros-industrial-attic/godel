# Boundary Data for a given blending surface (e.g. pocket)
# Sent as a series of 2D polygons (z-component is ignored) on XY plane of local frame.
# CCW polygons are external boundaries, CW polygons internal "obstacles"
geometry_msgs/Polygon[] boundaries

# Local frame of surface relative to planning frame. boundaries are expressed in this local frame.
geometry_msgs/Pose pose