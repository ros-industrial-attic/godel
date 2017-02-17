cmake_minimum_required(VERSION 2.8.3)
project(path_planning_plugins)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    eigen_conversions
    godel_msgs
    godel_process_path_generation
    path_planning_plugins_base
    pcl_ros
    roscpp
    tf
)

find_package(Eigen3 REQUIRED)

# Eigen 3.2 (Wily) only provides EIGEN3_INCLUDE_DIR, not EIGEN3_INCLUDE_DIRS
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

set(path_planning_plugins_SRCS
  src/openveronoi/blend_planner.cpp
  src/openveronoi/scan_planner.cpp
  src/profilometer/profilometer_scan.cpp
  src/mesh_importer/mesh_importer.cpp
)

set(path_planning_plugins_HDRS
  include/path_planning_plugins/openveronoi_plugins.h
  include/profilometer/profilometer_scan.h
  include/mesh_importer/mesh_importer.h
)

set(path_planning_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    ${path_planning_plugins_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    godel_msgs
    roscpp
    godel_process_path_generation
    path_planning_plugins_base
)

###########
## Build ##
###########

include_directories(${path_planning_plugins_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

## DescartesTrajectoryPt lib
add_library(${PROJECT_NAME}
    ${path_planning_plugins_SRCS}
    ${path_planning_plugins_HDRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
