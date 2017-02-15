cmake_minimum_required(VERSION 2.8.3)
project(path_planning_plugins)
add_definitions( -std=c++11 )

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    eigen_conversions
    godel_msgs
    godel_process_path_generation
    roscpp
    tf
)

find_package(Eigen REQUIRED)
find_package(PCL REQUIRED)

set(path_planning_plugins_SRCS
  src/path_planning_plugins.cpp
  src/profilometer_scan.cpp
  src/mesh_importer.cpp
)

set(path_planning_plugins_HDRS
  include/path_planning_base.h
  include/path_planning_plugins.h
  include/profilometer_scan.h
  include/mesh_importer.h
)

set(path_planning_plugins_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS
    ${path_planning_plugins_INCLUDE_DIRECTORIES}
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    godel_msgs
    roscpp
    godel_process_path_generation
)

###########
## Build ##
###########

include_directories(${path_planning_plugins_INCLUDE_DIRECTORIES}
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

## Declare a cpp library
add_library(${PROJECT_NAME}
  ${path_planning_plugins_SRCS}
  ${path_planning_plugins_HDRS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${PCL_LIBRARIES}
)


## MeshImporter library
add_library(mesh_importer
            src/mesh_importer.cpp
)

target_link_libraries(mesh_importer
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${PCL_LIBRARIES}
)


## ProfilometerScan library
add_library(profilometer_scan
            src/profilometer_scan.cpp
)

target_link_libraries(profilometer_scan
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${PCL_LIBRARIES}
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
