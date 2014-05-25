Godel Process Path Generation
==============

This package creates the high-level process path for surface blending.

# Running Visualization #

- Note: If package godel_polygon_offset is used, start its node: 
 - rosrun godel_polygon_offset godel_polygon_offset_node
- rosrun rviz rviz (use path_visualization.rviz in test)
- rosrun godel_process_path_generation process_visualization_node --demo

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
	 

# Classes #
**MeshImporter**

- Input: pcl::Mesh
- Actions
	- fitMeshToPlane
		- Plane represented by origin at COM of mesh, and plane normal
	- flattenToPlane(vector\<points/vertices>)
	- transformTo2D(vector\<points>)
	- fIndPointOnExternalBoundary(const &pcl::mesh)
	- extractExternalBoundary(const &pcl::mesh, etc)
- Member Variables
	- pcl::Mesh pointcloud
	- transform (calculated from FitMeshToPlane)
		- transform from global coordinates to local plane coordinates
	- vector\<pcl::points> boundary

