cmake_minimum_required(VERSION 2.8.3)
project(param_helpers)

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp
 DEPENDS yaml-cpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
