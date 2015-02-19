cmake_minimum_required(VERSION 2.8.3)
project(godel_path_planning)


find_package(catkin REQUIRED COMPONENTS
  godel_msgs
  godel_process_path_generation
  roscpp
  cmake_modules
  descartes_core
  descartes_moveit
  descartes_planner
  descartes_trajectory
  tf_conversions
)

find_package(Eigen REQUIRED)

ADD_DEFINITIONS("-std=c++0x") # Descartes core uses c++11 in header files

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES godel_path_planning
#  CATKIN_DEPENDS godel_msgs godel_process_path_generation roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

##_________
## Nodes ##
## Path Planner Node
add_executable(trajectory_planning_node
               src/trajectory_planning_node.cpp
               src/trajectory_planning.cpp
)
target_link_libraries(trajectory_planning_node
                      ${catkin_LIBRARIES}
                      ${Eigen_LIBRARIES}
)
add_dependencies(trajectory_planning_node
                 godel_msgs_generate_messages_cpp
)


#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_godel_path_planning.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
