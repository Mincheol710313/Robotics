cmake_minimum_required(VERSION 3.0.2)
project(agv_detect)

find_package(OpenCV 4 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  image_transport
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs image_transport 
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(aruco_detect src/aruco_detect.cpp)

target_link_libraries(
  aruco_detect
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
