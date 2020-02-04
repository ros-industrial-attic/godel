cmake_minimum_required(VERSION 2.8.3)
project(rapid_generator)

find_package(catkin REQUIRED)

include_directories(include)
add_library(rapid_generator src/rapid_emitter.cpp)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}/../..
)

install(TARGETS  rapid_generator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)