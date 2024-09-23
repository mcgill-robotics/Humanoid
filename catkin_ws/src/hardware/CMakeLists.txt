cmake_minimum_required(VERSION 3.0.2)
project(hardware)
set(MSG_DEP_SET geometry_msgs std_msgs humanoid_msgs)

find_package(catkin REQUIRED COMPONENTS
  ${MSG_DEP_SET}
  roscpp
  rospy
  std_msgs
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(	CATKIN_DEPENDS 
  ${MSG_DEP_SET}
  rospy
  tf
  tf2
  tf2_geometry_msgs
  tf2_ros
)