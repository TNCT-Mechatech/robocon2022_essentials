cmake_minimum_required(VERSION 3.0.2)
project(robocon2022_essentials_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs

  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  UserAction.msg
  Controller.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robocon2022_essentials_msgs
  CATKIN_DEPENDS
    roscpp 
    std_msgs 
    geometry_msgs
    message_runtime
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)