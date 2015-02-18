cmake_minimum_required(VERSION 2.8.3)
project(keyence_ljv_description)

find_package(catkin REQUIRED)

catkin_package()

foreach(dir config launch urdf)
  install(
    DIRECTORY ${dir}
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
endforeach()
