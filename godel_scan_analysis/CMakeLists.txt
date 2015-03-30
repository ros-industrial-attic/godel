cmake_minimum_required(VERSION 2.8.3)
project(godel_scan_analysis)

find_package(catkin REQUIRED COMPONENTS
  godel_keyence_ljv_driver
  keyence_ljv_description
  roscpp
)

catkin_package(
 INCLUDE_DIRS include
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

## Declare a cpp executable
add_executable(godel_scan_analysis_node src/godel_scan_analysis_node.cpp src/scan_roughness_scoring.cpp src/keyence_scan_server.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(godel_scan_analysis_node
  ${catkin_LIBRARIES}
)
