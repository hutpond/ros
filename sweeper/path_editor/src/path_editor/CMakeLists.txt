cmake_minimum_required(VERSION 3.5)
project(path_editor)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  add_definitions("-DDEBUG")
endif()

SET(CMAKE_C_COMPILER gcc-5)
SET(CMAKE_CXX_COMPILER g++-5)
add_compile_options(-std=c++11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
include_directories( ${catkin_INCLUDE_DIRS} )

# find dependencies
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Widgets SerialPort REQUIRED)

add_message_files (
  DIRECTORY
  msg

  FILES
  ads_ins_data.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  )

set(HEADERS
  src/QDrawPathWidget.h
  src/QOpenDriveObject.h
  src/QProjectObject.h
  src/QSettingDialog.h
  src/QEditorMainWindow.h
  src/QPanelWidget.h
  src/QReadDataObject.h
  )
set(SOURCES
  src/main.cpp
  src/QEditorMainWindow.cpp
  src/QPanelWidget.cpp
  src/QReadDataObject.cpp
  src/QDrawPathWidget.cpp
  src/QOpenDriveObject.cpp
  src/QProjectObject.cpp
  src/QSettingDialog.cpp
  )
set(RESOURCES
  src/planningpatheditor.qrc
  )
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} ${RESOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets Qt5::SerialPort)
#target_link_libraries(debug_tool zmq)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
