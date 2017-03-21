cmake_minimum_required(VERSION 2.8.3)
project(godel_msgs)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             actionlib
             actionlib_msgs
             genmsg
             geometry_msgs
             message_generation
             sensor_msgs
             trajectory_msgs
             visualization_msgs
)


#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'action' folder
add_action_files(
    DIRECTORY
    action
    FILES
    ProcessExecution.action
    ProcessPlanning.action
    SelectMotionPlan.action
)


## Generate messages in the 'msg' folder
add_message_files(
  FILES
  BlendingPlanParameters.msg
  ProcessPath.msg
  ProcessPlan.msg
  PathPlanningParameters.msg
  RobotScanParameters.msg
  SelectedSurfacesChanged.msg
  ScanPlanParameters.msg
  SurfaceBoundaries.msg
  SurfaceDetectionParameters.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  BlendingPlan.srv
  BlendProcessPlanning.srv
  EnsensoCommand.srv
  GetAvailableMotionPlans.srv
  KeyenceProcessPlanning.srv
  LoadSaveMotionPlan.srv
  OffsetBoundary.srv
  PathPlanning.srv
  RenameSurface.srv
  SelectMotionPlan.srv
  SelectSurface.srv
  SurfaceDetection.srv
  SurfaceBlendingParameters.srv
  TrajectoryExecution.srv
  TrajectoryPlanning.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES 
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  trajectory_msgs
)

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
  CATKIN_DEPENDS
  actionlib
  actionlib_msgs
  message_runtime
  geometry_msgs
  sensor_msgs
  genmsg
)


