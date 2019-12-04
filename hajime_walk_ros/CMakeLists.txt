cmake_minimum_required(VERSION 2.8.3)
project(hajime_walk_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
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
add_subdirectory(RTIMULib)

#set(ICE_DIR $ENV{ICE_HOME} CACHE PATH "ZeroC ICE installation dir(Default: $ICE_HOME)")

add_definitions(-DGANKENKUN_MODE)

add_definitions(-D_AFXDLL -DSIM -D_CRT_SECURE_NO_WARNINGS)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp
)

include_directories(
    include
	${ICE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}
	${BOOST_INCLUDE_DIRS}
	${Boost_INCLUDE_DIR}
	RTIMULib
    ${catkin_INCLUDE_DIRS}
)

link_directories(
	${ICE_DIR}/lib
	${Boost_LIBRARY_DIRS}
	${Boost_LIBRARY_DIR}
	RTIMULib
)

set(HR46_B3M_SRCS
src/acc.c                 src/calc_deg.c            src/calc_mv.c             
src/cntr.c                src/func.c                src/gyro.c
src/joy.c                 src/kine.c                src/main.cpp
src/motion.c              src/mvtbl.c               src/b3m.cpp
src/serv.c                src/servo_rs.c            src/serv_init.c
src/sq_motion.c           src/sq_ready.c            src/sq_start.c
src/sq_straight.c         src/sq_walk.c             src/var_init.c
src/pc_motion.cpp         src/OrientationEstimator.cpp

src/acc.h                 src/calc_deg.h            src/calc_mv.h
src/cntr.h                src/func.h                src/gyro.h
src/joy.h                 src/kine.h                src/motion.h
src/mvtbl.h               src/b3m.h                 src/serv.h
src/servo_rs.h            src/serv_init.h           src/sq_motion.h
src/sq_ready.h            src/sq_start.h            src/sq_straight.h
src/sq_walk.h             src/var.h                 src/pc_motion.h
src/OrientationEstimator.h                          src/KSerialPort.h
)


add_executable(hajime_walk
	${HR46_B3M_SRCS}
)

SET(hr46_b3m_LIBRARIES
	Ice
	IceUtil
)

target_link_libraries(hajime_walk
	${Boost_LIBRARIES}
	${hr46_b3m_LIBRARIES}
	RTIMULib
    ${catkin_LIBRARIES}
)


#SET(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_SOURCE_DIR}/build/)

#INSTALL(TARGETS hajime_walk
#  RUNTIME DESTINATION pyfiles
#  LIBRARY DESTINATION pyfiles
#  ARCHIVE DESTINATION pyfiles
#)

#install(FILES servo_para.txt DESTINATION pyfiles)
#install(FILES RTIMULib.ini DESTINATION pyfiles)

