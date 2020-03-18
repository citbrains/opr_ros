cmake_minimum_required(VERSION 2.8.3)
project(opr_gazebo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  roboticsgroup_gazebo_plugins
  roscpp
  geometry_msgs
  opr_msgs
)
catkin_package(CATKIN_DEPENDS roscpp)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(ground_truth_publisher src/ground_truth_publisher.cpp)
target_link_libraries(ground_truth_publisher ${catkin_LIBRARIES})
