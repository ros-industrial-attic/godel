cmake_minimum_required(VERSION 2.8.3)
project(blending_end_effector)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
