cmake_minimum_required(VERSION 3.0.2)
project(neuro_rover)

## Find catkin macros and libraries
## allowing us to use variables like ${CATKIN_PACKAGE_BIN_DESTINATION}
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

## Declare the package
catkin_package()

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Mark executable scripts (Python nodes) for installation
## This uses the variables defined by find_package above
catkin_install_python(PROGRAMS
  scripts/neuro_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)