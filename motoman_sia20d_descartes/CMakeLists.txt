cmake_minimum_required(VERSION 2.8.3)
project(motoman_sia20d_descartes)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
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
