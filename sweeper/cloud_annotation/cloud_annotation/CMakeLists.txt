cmake_minimum_required(VERSION 3.5)

project(cloud_annotation LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(/usr/include/pcl-1.8)
include_directories(/usr/include/eigen3)
include_directories(/usr/include/vtk-6.3)

find_package(Qt5 COMPONENTS Widgets REQUIRED)

add_executable(cloud_annotation
  main.cpp
  qcloudmainwnd.cpp
  qcloudmainwnd.h
  qcloudpoints.cpp
  qcloudpoints.h
  qpointsshowwidget.cpp
  qpointsshowwidget.h
  )

target_link_libraries(cloud_annotation PRIVATE Qt5::Widgets)
target_link_libraries(cloud_annotation PRIVATE boost_system)
target_link_libraries(cloud_annotation PRIVATE pcl_common)
target_link_libraries(cloud_annotation PRIVATE pcl_io)
#target_link_libraries(cloud_annotation PRIVATE opengl)

