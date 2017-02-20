cmake_minimum_required(VERSION 2.8.3)
project(godel_plugins)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  godel_msgs
  roscpp
  rqt_gui
  rqt_gui_cpp
  rviz
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

set(godel_plugins_SRCS
  src/rqt_plugins/robot_blending_plugin.cpp
  src/rviz_panels/robot_blending_panel.cpp
  src/widgets/robot_blending_widget.cpp
  src/widgets/path_planning_param_window.cpp
  src/widgets/parameter_window_base.cpp
  src/widgets/surface_detection_configuration.cpp
  src/widgets/robot_scan_configuration.cpp
  src/widgets/scan_tool_configuration_window.cpp
)

set(godel_plugins_HDRS
  include/godel_plugins/rqt_plugins/robot_blending_plugin.h
  include/godel_plugins/rviz_panels/robot_blending_panel.h
  include/godel_plugins/widgets/robot_blending_widget.h
  include/godel_plugins/widgets/path_planning_param_window.h
  include/godel_plugins/widgets/parameter_window_base.h
  include/godel_plugins/widgets/surface_detection_configuration.h
  include/godel_plugins/widgets/robot_scan_configuration.h
  include/godel_plugins/widgets/scan_tool_configuration_window.h
)

set(godel_plugins_UIS
  src/widgets/robot_blending_plugin.ui 
  src/widgets/robot_scan_configuration.ui
  src/widgets/pose_widget.ui
  src/widgets/surface_detection_configuration.ui
  src/widgets/path_planning_param_window.ui
  src/widgets/scan_tool_configuration_window.ui
)

set(godel_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS 
    ${godel_plugins_INCLUDE_DIRECTORIES}
  LIBRARIES 
    ${PROJECT_NAME}
  CATKIN_DEPENDS 
    godel_msgs
    roscpp
    rqt_gui 
    rqt_gui_cpp
    rviz
)

###########
## Build ##
###########
qt5_wrap_cpp(godel_plugins_MOCS ${godel_plugins_HDRS})
qt5_wrap_ui(godel_plugins_UIS_H ${godel_plugins_UIS})

include_directories(${godel_plugins_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    )

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${godel_plugins_UIS_H}
  ${godel_plugins_MOCS}
  ${godel_plugins_SRCS}
)

target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES} 
  Qt5::Widgets
  Qt5::Gui
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


