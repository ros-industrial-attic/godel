cmake_minimum_required(VERSION 2.8.3)
project(godel_surface_detection)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_ros
  roscpp
  sensor_msgs
  interactive_markers
  moveit_ros_move_group
  moveit_ros_planning_interface
  tf
  tf_conversions
  godel_process_path_generation
  godel_param_helpers
  godel_msgs
)
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS 
    include
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    pcl_msgs
    pcl_ros
    roscpp
    sensor_msgs
    interactive_markers
    tf
    tf_conversions
    godel_process_path_generation
    godel_param_helpers
    godel_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} 
  src/detection/surface_detection.cpp
  src/scan/robot_scan.cpp
  src/interactive/interactive_surface_server.cpp
  src/scan/profilimeter_scan.cpp
  src/services/trajectory_library.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} yaml-cpp)
add_dependencies(${PROJECT_NAME} godel_msgs_generate_messages_cpp)

## point cloud publisher node
add_executable(point_cloud_publisher_node src/nodes/point_cloud_publisher_node.cpp)
target_link_libraries(point_cloud_publisher_node ${PROJECT_NAME})

## point cloud generator node
add_executable(point_cloud_generator_node src/nodes/point_cloud_generator_node.cpp)
target_link_libraries(point_cloud_generator_node ${PROJECT_NAME})

## surface detection service
add_executable(surface_blending_service  src/services/surface_blending_service.cpp
                                         src/services/blending_service_path_generation.cpp)
target_link_libraries(surface_blending_service ${PROJECT_NAME})
add_dependencies(surface_blending_service godel_msgs_generate_messages_cpp)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
