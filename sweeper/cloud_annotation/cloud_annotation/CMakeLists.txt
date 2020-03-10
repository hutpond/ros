cmake_minimum_required(VERSION 3.5)

project(cloud_annotation LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(/usr/include/pcl-1.8)
include_directories(/usr/include/pcl-1.7)
include_directories(/usr/include/eigen3)

set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Widgets OpenGL REQUIRED)

add_executable(cloud_annotation
  main.cpp
  qcloudmainwnd.cpp
  qcloudmainwnd.h
  qcloudpoints.cpp
  qcloudpoints.h
  qpointsshowwidget.cpp
  qpointsshowwidget.h
  QHdMapWidget.cpp
  QHdMapWidget.h
  QAddSegmentDialog.cpp
  QAddSegmentDialog.h
  QAddSegmentDialog.ui
  QPointValue.cpp
  QPointValue.h
  QPointValue.ui
  QProjectDialog.cpp
  QProjectDialog.h
  QProjectDialog.ui
  cloud_annotation.qrc
)

target_link_libraries(cloud_annotation PRIVATE Qt5::Widgets)
target_link_libraries(cloud_annotation PRIVATE Qt5::OpenGL)
target_link_libraries(cloud_annotation PRIVATE boost_system)
target_link_libraries(cloud_annotation PRIVATE pcl_common)
target_link_libraries(cloud_annotation PRIVATE pcl_io)
target_link_libraries(cloud_annotation PRIVATE z)
target_link_libraries(cloud_annotation PRIVATE GL)
target_link_libraries(cloud_annotation PRIVATE GLU)
target_link_libraries(cloud_annotation PRIVATE glut)
