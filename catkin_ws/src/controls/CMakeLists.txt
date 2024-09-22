cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(	CATKIN_DEPENDS 
)