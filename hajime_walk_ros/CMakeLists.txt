cmake_minimum_required(VERSION 2.8.3)
project(hajime_walk_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  roslib
  opr_msgs
)

set(CMAKE_CXX_STANDARD 11)


set(CMAKE_CXX_FLAGS "-fPIC")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

set(Boost_USE_STATIC_LIBS        OFF)
set(Boost_USE_MULTITHREADED      ON)
set(Boost_USE_STATIC_RUNTIME    OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC")
set(CMAKE_CXX_LINK_EXECUTABLE "${CMAKE_CXX_LINK_EXECUTABLE} -ldl")

FIND_PACKAGE(Boost COMPONENTS system thread filesystem)
add_definitions(${Boost_LIB_DIAGNOSTIC_DEFINITIONS})


add_definitions(-DGANKENKUN_MODE)

add_definitions(-D_AFXDLL -DSIM -D_CRT_SECURE_NO_WARNINGS)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp sensor_msgs std_msgs
)

include_directories(
    include
	${CMAKE_CURRENT_SOURCE_DIR}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
)

link_directories(
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
)

set(HR46_B3M_SRCS
src/acc.c                 src/calc_deg.c            src/calc_mv.c             
src/cntr.c                src/func.c                src/gyro.c
src/joy.c                 src/kine.c                src/main.cpp
src/motion.c              src/mvtbl.c               src/serv.c                
src/serv_init.c           src/sq_motion.c           src/sq_ready.c            
src/sq_start.c            src/sq_straight.c         src/sq_walk.c             
src/var_init.c            src/pc_motion.cpp         
)

add_executable(hajime_walk
	${HR46_B3M_SRCS}
)

target_link_libraries(hajime_walk
	${Boost_LIBRARIES}
    ${catkin_LIBRARIES}
)


