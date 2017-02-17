cmake_minimum_required(VERSION 2.8.3)
project(meshing_plugins)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  godel_msgs
  meshing_plugins_base
  pcl_ros
  pluginlib
  roscpp
)

set(meshing_plugins_SRCS
  src/concave_hull_mesher.cpp
)

set(meshing_plugins_HDRS
  include/meshing_plugins/concave_hull_plugins.h
)

set(meshing_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    ${meshing_plugins_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    godel_msgs
    meshing_plugins_base
    pcl_ros
    roscpp
)

###########
## Build ##
###########

include_directories(${meshing_plugins_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${meshing_plugins_SRCS}
  ${meshing_plugins_HDRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} godel_msgs_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
