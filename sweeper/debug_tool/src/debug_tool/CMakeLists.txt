cmake_minimum_required(VERSION 3.5)
project(debug_tool)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs message_generation geometry_msgs debug_ads_msgs)
include_directories( ${catkin_INCLUDE_DIRS} )

# find dependencies
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(Qt5_DIR /opt/Qt5.9.5/5.9.5/gcc_64/lib/cmake/Qt5)
set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Widgets Svg REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/qwt/include)
link_directories(${CMAKE_CURRENT_SOURCE_DIR}/qwt/lib)

include_directories(/usr/include/eigen3)

add_message_files (
  FILES
  ads_PlanningData4Debug.msg
  ads_Radar28fTargetColl.msg
  ads_Radar73fTargetColl.msg
  ads_RadarTarget.msg
  ads_ReferencePoint.msg
  ads_Spline.msg
  ads_TargetPoint.msg
  ads_TrackTarget.msg
  ads_TrackTargetColl.msg
  ads_UltraSonicTarget.msg
  ads_UltraSonicTargetColl.msg
  ads_Vec4f.msg
  ads_planning_output.msg
  ads_pubcurb.msg
  ads_garbage_detection.msg
  ads_littersensor_data.msg
  ads_planning_trajectory.msg
  ads_Point.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )
catkin_package()

add_executable(debug_tool
  src/main.cpp
  src/GlobalDefine.cpp
  src/QBaseWidget.cpp
  src/QPlanningWidget.cpp
  src/QDebugToolMainWnd.cpp
  src/QPlanningParamWidget.cpp
  src/QBaseShowWidget.cpp
  src/QPlanningShowWidget.cpp
  src/QReadDataManagerRos.cpp
  src/QDataDisplayWidget.cpp
  src/QPlanningCostWidget.cpp
  src/QCostValueWidget.cpp
  src/QFullViewWidget.cpp
  src/QEditToolsWidget.cpp
  src/QNewPlanningWidget.cpp
  src/QNewPlanningShowWidget.cpp
  src/splines.cpp
  src/QNewPlanningPlot.cpp
  src/QBezierCurve.cpp
  src/QDecisionStateWidget.cpp
  src/qstateitem.cpp
  src/qstatewidget.cpp
  src/QFrameTimeWidget.cpp
  src/debugtool.qrc
  )

target_link_libraries(debug_tool jsoncpp)
target_link_libraries(debug_tool boost_system)
target_link_libraries(debug_tool boost_filesystem)
target_link_libraries(debug_tool qwt)

target_include_directories(debug_tool PUBLIC
  $<INSTALL_INTERFACE:include>)

add_dependencies(${PROJECT_NAME} debug_ads_msgs_gencpp)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Svg)

target_link_libraries(debug_tool ${catkin_LIBRARIES})

install(TARGETS debug_tool
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
