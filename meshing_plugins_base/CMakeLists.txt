cmake_minimum_required(VERSION 2.8.3)
project(meshing_plugins_base)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  pcl_ros
  roscpp
)

set(meshing_plugins_HDRS
  include/meshing_plugins_base/meshing_base.h
)

set(meshing_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    ${meshing_plugins_INCLUDE_DIRECTORIES}
  CATKIN_DEPENDS
    pcl_ros
    roscpp
)

###########
## Build ##
###########

include_directories(${meshing_plugins_INCLUDE_DIRECTORIES}
                    ${catkin_INCLUDE_DIRS}
)


#############
## Install ##
#############

install(
    DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
