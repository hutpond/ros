cmake_minimum_required(VERSION 3.5)
project(sweeper_hmi LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(Qt5_DIR /opt/Qt5.9.5/5.9.5/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED)

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()
include_directories( ${catkin_INCLUDE_DIRS} )

set(HEADERS
  src/ReadDataRos.h
  src/QSelfChecking.h
  )
set(SOURCES
  src/main.cpp
  src/ReadDataRos.cpp
  src/QSelfChecking.cpp
  )
set(RESOURCES
  src/sweeper_hmi.qrc
  )
set(QMLS
  src/main.qml
  src/subframes/Check.qml
  src/subframes/Header.qml
  src/subframes/Start.qml
  src/subframes/TaskSelect.qml
  src/subframes/UserPage.qml
  )
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} ${RESOURCES} ${QMLS})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
