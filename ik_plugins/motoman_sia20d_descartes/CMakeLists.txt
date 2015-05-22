cmake_minimum_required(VERSION 2.8.3)
project(motoman_sia20d_descartes)

add_definitions(-std=gnu++0x)
find_package(catkin REQUIRED COMPONENTS
  descartes_core
  descartes_moveit
  motoman_sia20d_ikfast_manipulator
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
    descartes_moveit
    motoman_sia20d_ikfast_manipulator
  DEPENDS
)

###########
## Build ##
###########

include_directories(include
                    ${catkin_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIRS}
                    ${Eigen_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
            src/motoman_sia20d_robot_model.cpp)
target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp
      test/motoman_sia20d_descartes_plugin.cpp)

  add_rostest_gtest(${PROJECT_NAME}_utest test/launch/utest.launch test/launch/motoman_sia20d_plugin.launch ${UTEST_SRC_FILES})
  target_link_libraries(${PROJECT_NAME}_utest ${catkin_LIBRARIES} ${Boost_LIBRARIES})
endif()
