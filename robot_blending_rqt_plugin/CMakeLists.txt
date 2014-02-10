cmake_minimum_required(VERSION 2.8.3)
project(robot_blending_rqt_plugin)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rqt_gui
  rqt_gui_cpp
)

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(robot_blending_rqt_plugin_UIS
	src/rqt_plugins/robot_blending_view.ui)

set(robot_blending_rqt_plugin_INCLUDE_DIRECTORIES
	include
	${CMAKE_CURRENT_BINARY_DIR})

catkin_package(
  INCLUDE_DIRS ${robot_blending_rqt_plugin_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp
)

###########
## Build ##
###########
#qt4_wrap_cpp(rqt_image_view_MOCS ${rqt_image_view_HDRS})
qt4_wrap_ui(robot_blending_rqt_plugin_UIS_H ${rqt_image_view_UIS})

include_directories(${robot_blending_rqt_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/robot_blending_rqt_plugin.cpp)

## Declare a cpp executable
# add_executable(robot_blending_rqt_plugin_node src/robot_blending_rqt_plugin_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(robot_blending_rqt_plugin_node robot_blending_rqt_plugin_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(robot_blending_rqt_plugin_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############


#############
## Testing ##
#############

