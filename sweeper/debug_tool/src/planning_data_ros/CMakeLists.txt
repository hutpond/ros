cmake_minimum_required(VERSION 3.5)
project(planning_data_ros LANGUAGES CXX)

SET(CMAKE_C_COMPILER gcc-5)
SET(CMAKE_CXX_COMPILER g++-5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation geometry_msgs)
include_directories( ${catkin_INCLUDE_DIRS} )

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
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
