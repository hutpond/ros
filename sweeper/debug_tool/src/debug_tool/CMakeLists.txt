cmake_minimum_required(VERSION 3.5)
project(debug_tool)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

SET(CMAKE_C_COMPILER gcc-5)
SET(CMAKE_CXX_COMPILER g++-5)
add_compile_options(-std=c++11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
include_directories( ${catkin_INCLUDE_DIRS} )
include_directories(/usr/local/pcl/include/pcl-1.9)
include_directories(/usr/include/eigen3)

# find dependencies
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Qt5_DIR /opt/Qt5.9.5/5.9.5/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

add_message_files (
  FILES
  PlanningData4Debug.msg
  ReferencePoint.msg
  TargetPoint.msg 
  Radar28fTargetColl.msg
  RadarTarget.msg
  TrackTargetColl.msg
  TrackTarget.msg
  UltraSonicTargetColl.msg
  UltraSonicTarget.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

add_executable(debug_tool src/main.cpp)

target_sources(debug_tool PRIVATE src/GlobalDefine.cpp)
target_sources(debug_tool PRIVATE src/QBaseWidget.cpp)
target_sources(debug_tool PRIVATE src/QLocationWidget.cpp)
target_sources(debug_tool PRIVATE src/QPerceptionShowWidget.cpp)
target_sources(debug_tool PRIVATE src/QPlanningWidget.cpp)
target_sources(debug_tool PRIVATE src/QDebugToolMainWnd.cpp)
target_sources(debug_tool PRIVATE src/QLocusWidget.cpp)
target_sources(debug_tool PRIVATE src/QPerceptionWidget.cpp)
target_sources(debug_tool PRIVATE src/QUltrasonicWidget.cpp)
target_sources(debug_tool PRIVATE src/QPerceptionWidget.cpp)
target_sources(debug_tool PRIVATE src/QAlgorithmWidget.cpp)
target_sources(debug_tool PRIVATE src/QErrorFigureWidget.cpp)
target_sources(debug_tool PRIVATE src/QPerceptionParamWidget.cpp)
target_sources(debug_tool PRIVATE src/QPlanningParamWidget.cpp)
target_sources(debug_tool PRIVATE src/QVarianceWidget.cpp)
target_sources(debug_tool PRIVATE src/QBaseShowWidget.cpp)
target_sources(debug_tool PRIVATE src/QEulerAngleWidget.cpp)
target_sources(debug_tool PRIVATE src/QPerceptionShow3DWidget.cpp)
target_sources(debug_tool PRIVATE src/QPlanningShowWidget.cpp)
#target_sources(debug_tool PRIVATE src/ReadDataManager.cpp)
target_sources(debug_tool PRIVATE src/QReadDataManagerRos.cpp)
target_sources(debug_tool PRIVATE src/QDataDisplayDialog.cpp)
target_sources(debug_tool PRIVATE src/DBAds.h)
target_sources(debug_tool PRIVATE src/ads_perception_export.h)

target_link_libraries(debug_tool jsoncpp)
target_link_libraries(debug_tool boost_system)
target_link_libraries(debug_tool boost_filesystem)

target_include_directories(debug_tool PUBLIC
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
#target_link_libraries(debug_tool zmq)

target_link_libraries(debug_tool ${catkin_LIBRARIES})

install(TARGETS debug_tool
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
