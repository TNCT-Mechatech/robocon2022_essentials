cmake_minimum_required(VERSION 3.0.2)
project(robocon2022_essentials_detection)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  robocon2022_essentials_msgs
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
