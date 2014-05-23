# Path Parameters
float64 tool_radius
float64 margin                  # distance to leave around the boundary
float64 overlap                 # overlap distance between adjacent paths
float64 safe_traverse_height    # height above surface to rise during rapid traversal moves

# Path Speeds
float64 approach_spd            # Speed to approach surface of part
float64 blending_spd            # Speed to perform blending
float64 retract_spd             # Speed to move away from surface of part
float64 traverse_spd            # Speed to travel while at traverse height

# Boundary Data
# Sent as a series of 2D polygons (z-component is ignored)
# CCW polygons are external boundaries, CW polygons internal "obstacles"
geometry_msgs/Polygon[] polygons