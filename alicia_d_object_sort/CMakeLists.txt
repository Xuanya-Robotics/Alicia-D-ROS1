cmake_minimum_required(VERSION 3.0.2)
project(alicia_d_object_sort)

## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  tf2_ros
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES alicia_d_object_sort
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)
