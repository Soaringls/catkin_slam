cmake_minimum_required(VERSION 3.0.2)
project(baidu_idl_car_canbus_pbcarstatus_msgs)

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
  ##需要的自定义msg
  aeb.msg    
  baidu_idl_car_canbus_pbcarstatus.msg
  battery.msg                           
  brake.msg            
  ems.msg                   
  epb.msg                   
  eps.msg   
  gas.msg   
  gear.msg  
  handshake.msg       
  license_new.msg     
  safety.msg                   
  ultrasonic.msg
  vehicle_spd.msg
)


## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)
catkin_package()
