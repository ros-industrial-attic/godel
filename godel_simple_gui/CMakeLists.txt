cmake_minimum_required(VERSION 2.8.3)
project(godel_simple_gui)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  godel_msgs
  roscpp
  rviz
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

catkin_package(
  CATKIN_DEPENDS godel_msgs roscpp rviz
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

set(godel_simple_gui_SRCS
# core
 src/blending_panel.cpp
 src/blending_widget.cpp
# states
 src/states/scan_teach_state.cpp
 src/states/scanning_state.cpp
 src/states/surface_select_state.cpp
 src/states/planning_state.cpp
 src/states/select_plans_state.cpp
 src/states/simulating_state.cpp
 src/states/wait_to_execute_state.cpp
 src/states/executing_state.cpp
 src/states/error_state.cpp
 src/states/select_all_surface_state.cpp
# options
 src/options_submenu.cpp
 src/parameter_window_base.cpp
 src/options/pose_widget.cpp
 src/options/robot_scan_configuration.cpp
 src/options/surface_detection_configuration.cpp
 src/options/scan_tool_configuration.cpp
 src/options/path_planning_configuration.cpp
)

set(godel_simple_gui_HDRS
  include/godel_simple_gui/blending_panel.h
  include/godel_simple_gui/blending_widget.h
  # states
  include/godel_simple_gui/gui_state.h
  include/godel_simple_gui/states/scan_teach_state.h
  include/godel_simple_gui/states/scanning_state.h
  include/godel_simple_gui/states/surface_select_state.h
  include/godel_simple_gui/states/planning_state.h
  include/godel_simple_gui/states/select_plans_state.h
  include/godel_simple_gui/states/simulating_state.h
  include/godel_simple_gui/states/wait_to_execute_state.h
  include/godel_simple_gui/states/executing_state.h
  include/godel_simple_gui/states/error_state.h
  include/godel_simple_gui/states/select_all_surface_state.h
  # options
  include/godel_simple_gui/options_submenu.h
  include/godel_simple_gui/parameter_window_base.h
  include/godel_simple_gui/options/pose_widget.h
  include/godel_simple_gui/options/robot_scan_configuration.h
  include/godel_simple_gui/options/surface_detection_configuration.h
  include/godel_simple_gui/options/scan_tool_configuration.h
  include/godel_simple_gui/options/path_planning_configuration.h
)

set(godel_simple_gui_UIS
 src/uis/blending_widget.ui
 src/uis/options_select_menu.ui
 src/uis/pose_widget.ui
 src/uis/robot_scan_configuration.ui
 src/uis/surface_detection_configuration.ui
 src/uis/scan_tool_configuration.ui
 src/uis/path_planning_configuration.ui
)

set(godel_simple_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(-DQT_NO_KEYWORDS)
qt5_wrap_cpp(godel_simple_gui_MOCS ${godel_simple_gui_HDRS})
qt5_wrap_ui(godel_simple_gui_UIS_H ${godel_simple_gui_UIS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${godel_simple_gui_INCLUDE_DIRECTORIES}
)

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${godel_simple_gui_UIS_H}
  ${godel_simple_gui_MOCS}
  ${godel_simple_gui_SRCS}
)

target_link_libraries(${PROJECT_NAME} 
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui 
                      )

add_dependencies(${PROJECT_NAME} godel_msgs_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

# Install
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
