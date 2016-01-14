cmake_minimum_required(VERSION 2.8.3)
project(godel_irb2400_support)

find_package(catkin REQUIRED COMPONENTS
  godel_robot_resources
  godel_surface_detection
)

catkin_package(
  CATKIN_DEPENDS 
    godel_robot_resources
    godel_surface_detection
)

install(DIRECTORY
  config
  launch
  rviz
  urdf
  yaml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
