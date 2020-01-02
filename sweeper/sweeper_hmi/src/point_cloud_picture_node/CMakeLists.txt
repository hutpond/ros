cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_picture_node)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs ads_msgs)

find_package(OpenCV REQUIRED)


###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  )

add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  point_cloud_picture
  )
