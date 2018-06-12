cmake_minimum_required(VERSION 2.8.3)
project(aruco_mapping2)

find_package(catkin REQUIRED COMPONENTS
  aruco
  aruco_msgs
  aruco_ros
  cv_bridge
  dynamic_reconfigure
  geometry_msgs
  image_transport
  roscpp
  rospy
  sensor_msgs
  tf
  visualization_msgs
  std_msgs
  message_generation
  nav_msgs
)

include_directories(${catkin_INCLUDE_DIRS}
                    ${PROJECT_SOURCE_DIR}/include/)



catkin_package(
  CATKIN_DEPENDS message_runtime
)