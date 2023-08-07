cmake_minimum_required(VERSION 3.0.2)
project(agv_detect)

find_package(OpenCV 4 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(realsense2 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp sensor_msgs image_transport cv_bridge
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
  ${cv_bridge_LIBRARIES}
)

add_executable(camera_pos_filter src/camera_pos_filter.cpp)

target_link_libraries(
  camera_pos_filter
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

add_executable(camera_tf_broadcaster src/camera_tf_broadcaster.cpp)

target_link_libraries(
  camera_tf_broadcaster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

add_executable(agv_tf_broadcaster src/agv_tf_broadcaster.cpp)

target_link_libraries(
  agv_tf_broadcaster
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)

add_executable(agv_tf_listener src/agv_tf_listener.cpp)

target_link_libraries(
  agv_tf_listener
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${cv_bridge_LIBRARIES}
)
