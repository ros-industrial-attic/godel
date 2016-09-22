cmake_minimum_required(VERSION 2.8.3)
project(abb_file_suite)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  trajectory_msgs
)

add_service_files(
  FILES
  ExecuteProgram.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
    include 
    rapid_generator/include
  LIBRARIES 
    ros_abb_ftp_interface
    rapid_generator
  CATKIN_DEPENDS 
    roscpp 
    trajectory_msgs 
    message_runtime
  DEPENDS
    curl
)

###########
## Build ##
###########

# Subproject 'rapid_generator'
add_subdirectory(rapid_generator)

include_directories(
  include
  rapid_generator/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(ros_abb_ftp_interface
 src/abb_motion_ftp_downloader.cpp
 src/ftp_upload.cpp
)

add_executable(ros_abb_ftp_interface_node src/abb_motion_ftp_downloader_node.cpp)

add_dependencies(ros_abb_ftp_interface abb_file_suite_generate_messages_cpp)
# add_dependencies(ros_abb_ftp_interface_node abb_file_suite_generate_messages_cpp)

target_link_libraries(ros_abb_ftp_interface
  rapid_generator
  curl
)

target_link_libraries(ros_abb_ftp_interface_node
 ros_abb_ftp_interface
 ${catkin_LIBRARIES}
)

#############
## Install ##
#############


# Mark executables and/or libraries for installation
install(TARGETS ros_abb_ftp_interface ros_abb_ftp_interface_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include rapid_generator/include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)