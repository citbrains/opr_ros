cmake_minimum_required(VERSION 2.8.3)
project(opr_kondo_driver)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  message_generation
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(opr_kondo_driver_v3 src/opr_kondo_driver_v3.cpp src/b3m.cpp)
target_link_libraries(opr_kondo_driver_v3 ${catkin_LIBRARIES} usb ftdi)
