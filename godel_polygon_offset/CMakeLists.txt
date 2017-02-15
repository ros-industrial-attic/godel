cmake_minimum_required(VERSION 2.8.3)
project(godel_polygon_offset)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  message_generation
  cmake_modules
  godel_process_path_generation
  godel_openvoronoi
  path_planning_plugins
)

find_package(Eigen REQUIRED)

find_package(Boost REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate services in the 'srv' folder
add_service_files(FILES
  OffsetPolygon.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  geometry_msgs
)

## Nothing is exposed here because this package is GPLv3 and other packages should remain Apache2
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES godel_polygon_offset
  CATKIN_DEPENDS geometry_msgs roscpp message_runtime godel_process_path_generation godel_openvoronoi
#  DEPENDS system_lib
)


###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

## Polygon Offset Library
add_library(godel_polygon_offset
            src/polygon_offset.cpp
)
target_link_libraries(godel_polygon_offset
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${Boost_LIBRARIES}
)

## polygon Offset Node
add_executable(godel_polygon_offset_node
               src/polygon_offset_node.cpp
)
target_link_libraries(godel_polygon_offset_node
                      ${catkin_LIBRARIES}
                      godel_polygon_offset
)
add_dependencies(godel_polygon_offset_node godel_polygon_offset_generate_messages_cpp)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS godel_polygon_offset godel_polygon_offset_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_godel_polygon_offset.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
