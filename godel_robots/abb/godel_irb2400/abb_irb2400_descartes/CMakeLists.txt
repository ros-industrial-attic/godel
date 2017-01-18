cmake_minimum_required(VERSION 2.8.12)
project(abb_irb2400_descartes)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  irb2400_ikfast_manipulator_plugin
  pluginlib
)

find_package(rosconsole_bridge REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}
  CATKIN_DEPENDS
    descartes_core
    descartes_moveit
    irb2400_ikfast_manipulator_plugin
    pluginlib
)

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)


add_library(${PROJECT_NAME}
            src/abb_irb2400_robot_model.cpp
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
