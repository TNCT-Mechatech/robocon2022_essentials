cmake_minimum_required(VERSION 3.0.2)
project(robocon2022_essentials_serial)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robocon2022_essentials_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES nuitrack_body_tracker
  CATKIN_DEPENDS 
    roscpp 
    geometry_msgs
    std_msgs 
    robocon2022_essentials_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(serial src/serial.cpp)
target_link_libraries(serial ${catkin_LIBRARIES})

target_link_libraries(serial ${CMAKE_CURRENT_SOURCE_DIR}/SerialBridge/bin/libSerialBridge.a)
target_include_directories(serial PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/SerialBridge/src)