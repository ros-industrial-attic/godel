cmake_minimum_required(VERSION 2.8.3)
project(godel_scan_analysis)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS roscpp pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(godel_scan_analysis_node src/godel_scan_analysis_node.cpp 
                                        src/scan_roughness_scoring.cpp 
                                        src/keyence_scan_server.cpp)

target_link_libraries(godel_scan_analysis_node
  ${catkin_LIBRARIES}
)
