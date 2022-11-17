cmake_minimum_required(VERSION 3.0.2)
project(robocon2022_essentials_visualizer)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
  geometry_msgs
  body_tracker_msgs
  robocon2022_essentials_msgs
)


set(_opencv_version 4)
find_package(OpenCV 4 QUIET COMPONENTS imgproc highgui)
if(NOT OpenCV_FOUND)
  message(STATUS "----------------Did not find OpenCV 4, trying OpenCV 3--------------")
  set(_opencv_version 3)
  find_package(OpenCV 3 REQUIRED COMPONENTS imgproc highgui)
endif()

catkin_package(
 LIBRARIES ${PROJECT_NAME}
 CATKIN_DEPENDS
  roscpp 
  sensor_msgs 
  std_msgs
  cv_bridge
  body_tracker_msgs
  robocon2022_essentials_msgs
 DEPENDS
 OpenCV
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(visualizer src/visualizer.cpp)
target_link_libraries(visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(gain_visualizer src/gain_visualizer.cpp)
target_link_libraries(gain_visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(controller_visualizer src/controller_visualizer.cpp)
target_link_libraries(controller_visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_executable(panel_visualizer src/panel_visualizer.cpp)
target_link_libraries(panel_visualizer
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
