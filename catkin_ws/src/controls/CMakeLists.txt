cmake_minimum_required(VERSION 3.0.2)
project(controls)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  humanoid_msgs
)

catkin_package(
  INCLUDE_DIRS src
  CATKIN_DEPENDS roscpp geometry_msgs std_msgs humanoid_msgs
)

add_executable(fast_mpc src/fast_mpc.cpp)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED jsoncpp)
target_include_directories(fast_mpc PRIVATE ${JSONCPP_INCLUDE_DIRS})
target_link_libraries(fast_mpc
  ${catkin_LIBRARIES} ${JSONCPP_LIBRARIES}
)

install(TARGETS fast_mpc
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(	CATKIN_DEPENDS 
)