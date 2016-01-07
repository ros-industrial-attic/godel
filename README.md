## Godel

Application for demonstrating surface blending with ros

Godel: Austrian logician and mathematician http://en.wikipedia.org/wiki/Kurt_G%C3%B6del

### Installation

- Install [wstool](http://wiki.ros.org/wstool) in order manage the repos inside the workspace
  ```
  sudo apt-get install python-wstool
  ```

- Cd into the 'src' directory of your catkin workspace and run the following:
  ```
  wstool init . 
  wstool merge https://github.com/ros-industrial-consortium/godel/raw/indigo-devel/godel.rosinstall
  wstool update
  rosdep install --from-paths . --ignore-src
  cd ..
  catkin_make
  ```

### Applications

- Run blending demo in full simulation mode (simulated robot and sensor)
  ```
  roslaunch godel_surface_detection robot_blending.launch
  ```

- Run blending demo in robot simulation mode (simulated robot and real sensor data)
  ```
  roslaunch godel_surface_detection robot_blending.launch sim_sensor:=false
  ```

- Run blending demo in sensor simulation mode (real robot and simulated sensor)
  ```
  roslaunch godel_surface_detection robot_blending.launch sim_robot:=false robot_ip:=[robot ip]
  ```

- Run blending demo in full real mode (real robot and sensor)
  ```
  roslaunch godel_surface_detection robot_blending.launch sim_sensor:=false 
  sim_robot:=false robot_ip:=[robot ip]
  ```

### Execution
After successfully generating a motion plan with Godel, a `trajectory.bag` file will written to disk inside the `ROS_HOME` directory, which defaults to `/home/<user>/.ros/`. 

To execute this file, open a new file, source the godel workspace, and run the following:
```
roslaunch godel_surface_detection trajectory_test.launch bagfile:=trajectory.bag change_speed:=true speed:=0.1
```

Replace the '0.1' value with whatever linear tool speed you desire.

