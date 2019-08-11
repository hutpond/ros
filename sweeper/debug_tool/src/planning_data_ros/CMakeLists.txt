cmake_minimum_required(VERSION 3.5)
project(planning_data_ros LANGUAGES CXX)

SET(CMAKE_C_COMPILER gcc-5)
SET(CMAKE_CXX_COMPILER g++-5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
include_directories( ${catkin_INCLUDE_DIRS} )

add_message_files (
  FILES
  PlanningData4Debug.msg
  ReferencePoint.msg
  TargetPoint.msg
  Radar28fTargetColl.msg
  Radar73fTargetColl.msg
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

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
