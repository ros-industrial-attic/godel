cmake_minimum_required(VERSION 2.8.3)
project(godel_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs sensor_msgs visualization_msgs genmsg)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   FILES
   RobotScanParameters.msg
   SurfaceDetectionParameters.msg
   SelectedSurfacesChanged.msg
   BlendingPlanParameters.msg
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   SurfaceDetection.srv
   SelectSurface.srv
   BlendingPlan.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs visualization_msgs)

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
  CATKIN_DEPENDS message_runtime geometry_msgs sensor_msgs genmsg
)


