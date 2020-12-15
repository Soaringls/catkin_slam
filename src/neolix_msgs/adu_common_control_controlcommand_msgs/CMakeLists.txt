cmake_minimum_required(VERSION 3.0.2)
project(adu_common_control_controlcommand_msgs)

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
  adu_common_control_controlcommand.msg
  control_header.msg           
  lat_ctrl_ctx.msg
  contrl_context.msg                     
  depended_modules.msg                   
  long_ctrl_ctx.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package()
