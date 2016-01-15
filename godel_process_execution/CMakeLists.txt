cmake_minimum_required(VERSION 2.8.3)
project(godel_process_execution)

find_package(catkin REQUIRED COMPONENTS
  abb_file_suite
  godel_msgs
  industrial_robot_simulator_service
  trajectory_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
    abb_file_suite
    godel_msgs
    industrial_robot_simulator_service
    trajectory_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(abb_blend_process_service_node 
  src/abb_blend_process_service_node.cpp 
  src/abb_blend_process_service.cpp
  src/process_utils.cpp
)

add_executable(blend_process_service_node
  src/blend_process_service_node.cpp 
  src/blend_process_service.cpp 
  src/process_utils.cpp
)

add_executable(keyence_process_service_node 
  src/keyence_process_service_node.cpp 
  src/keyence_process_service.cpp
  src/process_utils.cpp
)

add_dependencies(abb_blend_process_service_node godel_msgs_generate_messages_cpp)
add_dependencies(abb_blend_process_service_node industrial_robot_simulator_service_generate_messages_cpp)

add_dependencies(blend_process_service_node godel_msgs_generate_messages_cpp)
add_dependencies(blend_process_service_node industrial_robot_simulator_service_generate_messages_cpp)

add_dependencies(keyence_process_service_node godel_msgs_generate_messages_cpp)
add_dependencies(keyence_process_service_node industrial_robot_simulator_service_generate_messages_cpp)


target_link_libraries(abb_blend_process_service_node
  ${catkin_LIBRARIES}
)

target_link_libraries(blend_process_service_node
  ${catkin_LIBRARIES}
)

target_link_libraries(keyence_process_service_node
  ${catkin_LIBRARIES}
)
