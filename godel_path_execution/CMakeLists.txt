cmake_minimum_required(VERSION 2.8.3)
project(godel_path_execution)

find_package(catkin REQUIRED COMPONENTS
  godel_msgs
  roscpp
  industrial_robot_simulator_service
  actionlib
)

catkin_package(
  CATKIN_DEPENDS
    actionlib
    godel_msgs 
    roscpp 
    industrial_robot_simulator_service
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(path_execution_service_node 
  src/path_execution_service_node.cpp 
  src/path_execution_service.cpp
)

add_dependencies(path_execution_service_node godel_msgs_generate_messages_cpp)
add_dependencies(path_execution_service_node industrial_robot_simulator_service_generate_messages_cpp)

target_link_libraries(path_execution_service_node
  ${catkin_LIBRARIES}
)

install(TARGETS path_execution_service_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
