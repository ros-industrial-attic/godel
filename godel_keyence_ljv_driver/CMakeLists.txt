cmake_minimum_required(VERSION 2.8.3)
project(godel_keyence_ljv_driver)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  simple_message
  pcl_ros
  dynamic_reconfigure
)

find_package(Boost COMPONENTS signals)

add_definitions(-DROS=1)
add_definitions(-DLINUXSOCKETS=1)

generate_dynamic_reconfigure_options(cfg/Keyence.cfg)

catkin_package(
  CATKIN_DEPENDS roscpp simple_message pcl_ros
)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(driver_node src/driver.cpp src/ljv7_rawdata.cpp)
target_link_libraries(driver_node simple_message ${catkin_LIBRARIES})
add_dependencies(driver_node ${PROJECT_NAME}_gencfg)

install(TARGETS driver_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
