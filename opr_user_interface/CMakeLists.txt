cmake_minimum_required(VERSION 2.8.3)
project(opr_user_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rviz
    opr_msgs
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)
add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(MOC_FILES
    include/husendcom.hpp
)

qt5_wrap_cpp(MOC_FILES_GUI
    include/statusui.hpp
)

qt5_wrap_cpp(MOC_FILES_RVIZ
    include/panel.hpp
)

set(SOURCE_FILES
    src/husendcom.cpp
    ${MOC_FILES}
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/panel.cpp ${MOC_FILES_RVIZ} ${SOURCE_FILES})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_LIBRARIES})

add_executable(statusUI src/statusui.cpp ${MOC_FILES_GUI} ${SOURCE_FILES})
target_link_libraries(statusUI ${catkin_LIBRARIES} ${QT_LIBRARIES})
