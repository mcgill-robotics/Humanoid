cmake_minimum_required(VERSION 3.0.2)
project(bringup)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  controls	
  unity
  hardware
  rosserial_arduino
  rosserial_client
)

catkin_package(	CATKIN_DEPENDS 
  controls	
  unity
  hardware
  rosserial_arduino
  rosserial_client
)

rosserial_generate_ros_lib(
	PACKAGE rosserial_arduino
	SCRIPT make_libraries.py
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)