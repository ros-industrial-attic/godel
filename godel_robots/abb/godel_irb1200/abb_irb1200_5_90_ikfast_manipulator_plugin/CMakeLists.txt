cmake_minimum_required(VERSION 2.8.3)
project(abb_irb1200_5_90_ikfast_manipulator_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  moveit_core
  pluginlib
  roscpp
  tf_conversions
  eigen_conversions
)

include_directories(${catkin_INCLUDE_DIRS})

catkin_package(
  LIBRARIES
  CATKIN_DEPENDS
    moveit_core
    pluginlib
    roscpp
    tf_conversions
    eigen_conversions
)

include_directories(include)

find_package(LAPACK REQUIRED)

add_library(abb_irb1200_5_90_manipulator_moveit_ikfast_plugin src/abb_irb1200_5_90_manipulator_ikfast_moveit_plugin.cpp)
target_link_libraries(abb_irb1200_5_90_manipulator_moveit_ikfast_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

add_library(abb_irb1200_5_90_manipulator_asus_moveit_ikfast_plugin src/abb_irb1200_5_90_manipulator_asus_ikfast_moveit_plugin.cpp)
target_link_libraries(abb_irb1200_5_90_manipulator_asus_moveit_ikfast_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${LAPACK_LIBRARIES})

install(TARGETS abb_irb1200_5_90_manipulator_moveit_ikfast_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(TARGETS abb_irb1200_5_90_manipulator_asus_moveit_ikfast_plugin LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(
  FILES
  abb_irb1200_5_90_manipulator_moveit_ikfast_plugin_description.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
