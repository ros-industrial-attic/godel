cmake_minimum_required(VERSION 2.8.3)
project(simulator_service)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  control_msgs
  roscpp
  trajectory_msgs
)

add_service_files(
  FILES
  SimulateTrajectory.srv
)

generate_messages(
  DEPENDENCIES
  control_msgs
  trajectory_msgs 
)

catkin_package(
  CATKIN_DEPENDS 
  control_msgs
  roscpp
  trajectory_msgs 
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

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS simulator_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install( DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
