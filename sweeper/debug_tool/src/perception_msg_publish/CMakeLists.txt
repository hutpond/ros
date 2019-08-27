cmake_minimum_required(VERSION 3.5)
project(perception_msg_publish LANGUAGES CXX)

SET(CMAKE_C_COMPILER gcc-5)
SET(CMAKE_CXX_COMPILER g++-5)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
include_directories( ${catkin_INCLUDE_DIRS} )

add_message_files (
  FILES
  TrackTargetColl.msg
  TrackTarget.msg
  UltrasonicTargetColl.msg
  UltrasonicTarget.msg
  msflOutput.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime)

add_executable(${PROJECT_NAME} src/main.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/application.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/read_files.cpp)
target_sources(${PROJECT_NAME} PRIVATE src/message_publish.cpp)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

add_dependencies(${PROJECT_NAME} perception_msg_publish_gencpp)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} jsoncpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
