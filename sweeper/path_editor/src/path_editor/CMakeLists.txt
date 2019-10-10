cmake_minimum_required(VERSION 3.5)
project(path_editor)

if ("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
  add_definitions("-DDEBUG")
endif()

add_compile_options(-std=c++11)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(/usr/local/protobuf/include)
link_directories(/usr/local/protobuf/lib)

include_directories(/usr/local/glog/include)
link_directories(/usr/local/glog/lib)

include_directories(/usr/local/gflags/include)
link_directories(/usr/local/gflags/lib)

include_directories(/usr/local/glog/include)
include_directories(/usr/include/eigen3)
include_directories(/usr/local/gflags/include)

include_directories(/usr/local/hdmap/include)
link_directories(/usr/local/hdmap/lib)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)
include_directories( ${catkin_INCLUDE_DIRS} )

# find dependencies
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

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
  src/QProjectObject.h
  src/QProjectManagerWidget.h
  src/QSettingDialog.h
  src/QEditorMainWindow.h
  src/QPanelWidget.h
  src/QReadDataRosObject.h
  src/gps.h
  src/QAddElementDialog.h
  )
set(SOURCES
  src/main.cpp
  src/QEditorMainWindow.cpp
  src/QPanelWidget.cpp
  src/QReadDataRosObject.cpp
  src/QDrawPathWidget.cpp
  src/QProjectManagerWidget.cpp
  src/QProjectObject.cpp
  src/QSettingDialog.cpp
  src/gps.cpp
  src/QAddElementDialog.cpp
  )
set(RESOURCES
  src/planningpatheditor.qrc
  )
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} ${RESOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME} hdmap)
target_link_libraries(${PROJECT_NAME} hdmap_util)
target_link_libraries(${PROJECT_NAME} protobuf)
target_link_libraries(${PROJECT_NAME} glog)
target_link_libraries(${PROJECT_NAME} gflags)
target_link_libraries(${PROJECT_NAME} pthread)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
