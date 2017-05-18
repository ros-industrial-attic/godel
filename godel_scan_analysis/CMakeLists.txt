cmake_minimum_required(VERSION 2.8.3)
project(godel_scan_analysis)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    pcl_ros
    roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

# Defines some tools for dealing with laser profile data
add_library(${PROJECT_NAME}
  src/scan_aggregator/profile_scan_aggregator.cpp
  src/scan_aggregator/profile_scan_fusion.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Defines a node that aggregates clouds & scores their roughness
add_executable(godel_scan_analysis_node 
  src/godel_scan_analysis_node.cpp
  src/scan_roughness_scoring.cpp
  src/keyence_scan_server.cpp
)

target_link_libraries(godel_scan_analysis_node
  ${catkin_LIBRARIES}
)

# Define another test node for new stuff
add_executable(godel_scan_fusion_node
  src/scan_fusion_test_node.cpp
)

target_link_libraries(godel_scan_fusion_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(TARGETS godel_scan_analysis_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
