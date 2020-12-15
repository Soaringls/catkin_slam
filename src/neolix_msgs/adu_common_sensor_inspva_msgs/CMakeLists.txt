cmake_minimum_required(VERSION 3.0.2)
project(adu_common_sensor_inspva_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  message_generation
  sensor_msgs
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ##adu::common::sensor::INSPVA需要的自定义msg
  adu_common_sensor_inspva.msg
  novatel_header.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package()
