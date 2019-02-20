cmake_minimum_required(VERSION 2.8.3)
project(sigverse_pr2_control)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  control_msgs
  controller_manager
  robot_state_publisher
  roscpp
  sensor_msgs
  joint_limits_interface
  pr2_description
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(sigverse_pr2_control src/sigverse_pr2_control.cpp src/sigverse_pr2_hw.cpp)
target_link_libraries(sigverse_pr2_control ${catkin_LIBRARIES})
