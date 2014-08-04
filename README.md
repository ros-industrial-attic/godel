Godel
==============

Application for demonstrating surface blending with ros

Godel: Austrian logician and mathematician http://en.wikipedia.org/wiki/Kurt_G%C3%B6del


### Installation
==============

Godel can be installed from source by cloning this repo.  Package specific dependencies/installation instructions can be found below.

 * [Polygon Offset Package](godel_polygon_offset/README.md)
 * [Process Path Generation](godel_process_path_generation/README.md) 

### Instructions
==============


##### - Run blending demo in full simulation mode (simulated robot and sensor)
```
roslaunch godel_surface_detection robot_blending.launch
```

##### - Run blending demo in robot simulation mode (simulated robot and real sensor data)
```
roslaunch godel_surface_detection robot_blending.launch sim_sensor:=false
```

##### - Run blending demo in sensor simulation mode (real robot and simulated sensor)
```
roslaunch godel_surface_detection robot_blending.launch sim_robot:=false robot_ip:=[robot ip]
```

##### - Run blending demo in full real mode (real robot and sensor)
```
roslaunch godel_surface_detection robot_blending.launch sim_sensor:=false 
sim_robot:=false robot_ip:=[robot ip]
```
