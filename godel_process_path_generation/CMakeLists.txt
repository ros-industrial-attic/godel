cmake_minimum_required(VERSION 2.8.3)
project(godel_process_path_generation)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
             roscpp
             pcl_ros
             godel_msgs
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED QUIET)
find_package(Eigen REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
               INCLUDE_DIRS include
#  LIBRARIES godel_process_path_generation
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(mesh_importer
            src/mesh_importer.cpp
)
target_link_libraries(mesh_importer
					            ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
                      ${PCL_LIBRARIES}
)

add_library(process_path
            src/process_path.cpp
            src/process_pt.cpp
            src/process_transition.cpp
)
target_link_libraries(process_path
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)

add_library(process_path_generator
            src/process_path_generator.cpp
)
target_link_libraries(process_path_generator
                      process_path
)

add_executable(process_path_generator_node
               src/process_path_generator_node.cpp
)
target_link_libraries(process_path_generator_node
                      process_path
                      process_path_generator
)
add_dependencies(process_path_generator_node godel_msgs_generate_messages_cpp)

add_executable(process_visualization_node
               src/process_path_visualization_node
)
target_link_libraries(process_visualization_node
                      ${catkin_LIBRARIES}
)
add_dependencies(process_visualization_node godel_msgs_generate_messages_cpp)


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
# install(TARGETS godel_process_path_generation godel_process_path_generation_node
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
catkin_add_gtest(test_MeshImporter test/test_MeshImporter.cpp)
target_link_libraries(test_MeshImporter
                      mesh_importer
)

catkin_add_gtest(test_ProcessPathGenerator test/test_ProcessPathGenerator.cpp)
target_link_libraries(test_ProcessPathGenerator
                      process_path_generator
)

# if(TARGET test_MeshImporter)
#   target_link_libraries(test_MeshImporter ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
