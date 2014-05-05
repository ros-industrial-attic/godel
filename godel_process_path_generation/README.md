Godel Process Path Generation
==============

This package creates the high-level process path for surface blending.

# Input #

- Pointcloud of a surface.
	- Outside edge of surface is assumed to be boundary.
	- Internal edges are currently ignored, but will be incorporated as features in the future.
- Tool radius or diameter
- Desired offset for each loop (absolute)
- Safe height for rapid transit moves

# Output #

- ProcessPlan for the entire surface.
	- ProcessPlan should be sent to TrajectoryPlanner for trajectory planning.


