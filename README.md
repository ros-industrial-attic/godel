## Godel

Application for demonstrating surface blending with ROS.

Godel: Austrian logician and mathematician http://en.wikipedia.org/wiki/Kurt_G%C3%B6del

### Installation

- Install [wstool](http://wiki.ros.org/wstool) in order manage the repos inside the workspace
  ```
  sudo apt install python-wstool
  ```

- Cd into the 'src' directory of your catkin workspace and run the following:
  ```
  wstool init . 
  wstool merge https://github.com/ros-industrial-consortium/godel/raw/kinetic-devel/godel.rosinstall
  wstool update
  rosdep install --from-paths . --ignore-src
  ```

- Finally, to build:
  ```
  catkin build
  ```

### Applications

- Run blending demo in full simulation mode (simulated robot and sensor)
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch
  ```
  Run the simulation with real point cloud data:
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch real_pcd:=true pcd_location:=/path/to/file.pcd
  ```
  Download pcd files and unzip in your HOME directory: https://s3-us-west-2.amazonaws.com/godelscanfiles/godel_point_cloud_data.zip

- Run blending demo in robot simulation mode (simulated robot and real sensor data)
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_sensor:=false
  ```

- Run blending demo in sensor simulation mode (real robot and simulated sensor)
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_robot:=false robot_ip:=[robot ip]
  ```

- Run blending demo in full real mode
  ```
  roslaunch godel_irb2400_support irb2400_blending.launch sim_sensor:=false 
  sim_robot:=false robot_ip:=[robot ip]
  ```

### Qt Glyph Loading Segfault (Kinetic)

- Rviz on Kinetic is prone to a segmentation fault caused by internal functions in the Qt library. Our current work-around is to set the following environment variable:
  ```
  export QT_NO_FT_CACHE=1
  ```

### Planning and Meshing Plugins

- The `meshing_plugins_base` and `path_planning_plugins_base` packages define the interfaces that Godel uses to both perform surface reconstruction
  and tool path generation based on those reconstructions.

- The current default mesher works well for point clouds regions that can be accurately modeled by a convex polygon. The default blending tool planner only works well for
  planar meshes.

- We are working on an experimental library for more arbitrary surface reconstruction and tool path planning in the [noether package](https://github.com/ros-industrial/noether.git).
  - To use, first install the dependencies of noether, which include VTK 7.1 and PCL 1.8.
  - Clone the [noether package](https://github.com/ros-industrial/noether.git) into your workspace.
  - Clone the [godel_noether](https://github.com/Jmeyer1292/godel_noether) package in your workspace.
  - In your godel robot support package, e.g. `godel_irb2400_support`, modify the config/plugins.yaml file to read:
  ```
    meshing_plugin_name: "godel_noether::NoetherMesher"
    blend_tool_planning_plugin_name: "godel_noether::NoetherPathPlanner"
  ```

### Keyence Laser Scanner
- To run the keyence laser scanner driver (replace `KEYENCE_CONTROLLER_IP` with the ip-address of your sensor):
  ```
  rosrun keyence_experimental keyence_driver_node _controller_ip:=KEYENCE_CONTROLLER_IP _frame_id:=keyence_sensor_optical_frame
  ```
  
  - If you have issues connecting, ensure that the IP address matches that of the controller and ensure that your computer is on the same subnet.

- To acquire laser scans and score them, run the following (replace `VOXEL_SIZE_IN_METERS` with your desired voxel size):
  ```
  roslaunch godel_scan_analysis scan_analysis.launch world_frame:=world_frame scan_frame:=keyence_sensor_optical_frame voxel_leaf_size:=VOXEL_SIZE_IN_METERS
  ```
