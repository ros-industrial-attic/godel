cmake_minimum_required(VERSION 2.8.3)
project(path_planning_plugins_base)

add_compile_options( -std=c++11 )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    pcl_ros
    roscpp
)

set(path_planning_plugins_HDRS
  include/path_planning_plugins_base/path_planning_base.h
)

set(path_planning_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    ${path_planning_plugins_INCLUDE_DIRECTORIES}
  CATKIN_DEPENDS
    geometry_msgs
    pcl_ros
    roscpp
)

###########
## Build ##
###########

include_directories(
    ${path_planning_plugins_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
