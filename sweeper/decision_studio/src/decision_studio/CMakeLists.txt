cmake_minimum_required(VERSION 2.8.3)
project(decision_studio)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp std_msgs geometry_msgs message_generation
  )
set(Qt5_DIR /opt/Qt5.12.4/5.12.4/gcc_64/lib/cmake/Qt5)
find_package(Qt5 COMPONENTS Core Gui Widgets Svg REQUIRED)

add_message_files(
  DIRECTORY
  msg
  FILES
  ads_DecisionData4Debug.msg
  ads_TrackTarget.msg
  ads_ReferencePoint.msg
  ads_Spline.msg
  ads_Vec4f.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES decision_studio
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME}_node
  src/main.cpp
  src/decision_subscriber.h
  src/decision_subscriber.cpp
  src/qstudiowindow.h
  src/qstudiowindow.cpp
  src/qcentralwidget.h
  src/qcentralwidget.cpp
  src/qbaseshowwidget.h
  src/qbaseshowwidget.cpp
  src/qshowwidget.h
  src/qshowwidget.cpp
  src/qstatewidget.h
  src/qstatewidget.cpp
  src/qstateitem.h
  src/qstateitem.cpp
  src/QBezierCurve.h
  src/QBezierCurve.cpp
  src/qreplaywidget.h
  src/qreplaywidget.cpp
  src/decision_studio.qrc
  )

# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node Qt5::Core Qt5::Gui Qt5::Widgets Qt5::Svg)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  )

target_include_directories(${PROJECT_NAME}_node PUBLIC
  $<INSTALL_INTERFACE:include>)


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_decision_studio.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
