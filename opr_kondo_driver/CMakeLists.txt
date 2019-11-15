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

add_executable(kondo_driver src/kondo_driver.cpp src/b3m.c)
target_link_libraries(kondo_driver ${catkin_LIBRARIES} usb ftdi)
