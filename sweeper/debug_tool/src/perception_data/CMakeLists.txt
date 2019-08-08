cmake_minimum_required(VERSION 2.8.3)
project(perception_data)

add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(perception_data src/main.cpp)
target_link_libraries(perception_data ${catkin_LIBRARIES})
target_link_libraries(perception_data jsoncpp)
