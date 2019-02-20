cmake_minimum_required(VERSION 2.8.3)
project(sigverse_hsr_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  controller_manager
  robot_state_publisher
  roscpp
  rospy
  sensor_msgs
  joint_limits_interface
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sigverse_hsr_control src/sigverse_hsr_control.cpp src/sigverse_hsr_hw.cpp)
target_link_libraries(sigverse_hsr_control ${catkin_LIBRARIES})
