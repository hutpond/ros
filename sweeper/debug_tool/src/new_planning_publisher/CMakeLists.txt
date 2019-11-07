cmake_minimum_required(VERSION 2.8.3)
project(new_planning_publisher)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
  roscpp std_msgs message_generation debug_ads_msgs)
include_directories( ${catkin_INCLUDE_DIRS} )

generate_messages(
  DEPENDENCIES
  std_msgs
  )

catkin_package(
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME} src/main.cpp)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME} debug_ads_msgs_gencpp)

