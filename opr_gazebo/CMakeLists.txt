cmake_minimum_required(VERSION 2.8.3)
project(opr_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_ros_control
  roboticsgroup_gazebo_plugins
)
catkin_package()

