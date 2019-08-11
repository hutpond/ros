cmake_minimum_required(VERSION 3.5)
project(sweeper_hmi LANGUAGES CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set(CMAKE_C_COMPILER gcc-7)
#set(CMAKE_CXX_COMPILER g++-7)

set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Qml Quick REQUIRED)

# ros
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package()
include_directories( ${catkin_INCLUDE_DIRS} )

set(HEADERS
  src/QDataManager.h
  src/dbAdsApi4HMI.h
  )
set(SOURCES
  src/main.cpp
  src/QDataManager.cpp
  src/dbAdsApi4HMI.cpp
  )
set(RESOURCES
  src/sweeper_hmi.qrc
  )
add_executable(${PROJECT_NAME} ${HEADERS} ${SOURCES} ${RESOURCES})

target_include_directories(${PROJECT_NAME} PUBLIC
  $<INSTALL_INTERFACE:include>)

target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<OR:$<CONFIG:Debug>,$<CONFIG:RelWithDebInfo>>:QT_QML_DEBUG>)
target_link_libraries(${PROJECT_NAME} Qt5::Core Qt5::Gui Qt5::Qml Qt5::Quick)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
