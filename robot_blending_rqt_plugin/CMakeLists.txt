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

set(robot_blending_rqt_plugin_SRCS
	src/rqt_plugins/robot_blending_plugin.cpp)

set(robot_blending_rqt_plugin_HDRS
	include/robot_blending_rqt_plugin/rqt_plugins/robot_blending_plugin.h)

set(robot_blending_rqt_plugin_UIS
	src/rqt_plugins/robot_blending_plugin.ui)

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
qt4_wrap_cpp(robot_blending_rqt_plugin_MOCS ${robot_blending_rqt_plugin_HDRS})
qt4_wrap_ui(robot_blending_rqt_plugin_UIS_H ${robot_blending_rqt_plugin_UIS})

include_directories(${robot_blending_rqt_plugin_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

## Declare a cpp library
 add_library(${PROJECT_NAME}
	${robot_blending_rqt_plugin_UIS_H}
	${robot_blending_rqt_plugin_MOCS}
	${robot_blending_rqt_plugin_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

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
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


