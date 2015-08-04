cmake_minimum_required(VERSION 2.8.3)
project(simulator_service)

find_package(catkin REQUIRED COMPONENTS
  industrial_robot_simulator
  roscpp
  trajectory_msgs
  control_msgs
  actionlib
)

add_service_files(
  FILES
  SimulateTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  trajectory_msgs control_msgs
)

catkin_package(
 CATKIN_DEPENDS industrial_robot_simulator roscpp
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(simulator_service_node src/simulator_service_node.cpp)

add_dependencies(simulator_service_node simulator_service_generate_messages_cpp)

target_link_libraries(simulator_service_node
  ${catkin_LIBRARIES}
)
