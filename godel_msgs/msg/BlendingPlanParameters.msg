# Path Parameters
float64 tool_radius           # (m) actual radius of blending tool
float64 margin                # (m) additional distance to leave near surface boundaries
float64 overlap               # (m) overlap distance between adjacent paths

# Process Parameters
float64 spindle_speed         # (rpm) Spindle Speed
float64 tool_force            # (kg)  Downward force to apply

# Path Speeds
float64 approach_spd          # (m/s) Speed to approach surface of part
float64 blending_spd          # (m/s) Speed to perform blending
float64 retract_spd           # (m/s) Speed to move away from surface of part
float64 traverse_spd          # (m/s) Speed to travel while at traverse height

# Misc
float64 discretization        # (m) How densely to space adjacent Cartesian points
float64 safe_traverse_height  # (m) height above surface to rise during rapid traversal moves
float64 z_adjust              # (m) height adjustment along surface normal to adjust for different tools

# Pre-Process Surface Parameters
float64 min_boundary_length   # (m) Boundaries below threshold are ignored during process path creation
