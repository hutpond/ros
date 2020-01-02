cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_picture)

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

add_library(${PROJECT_NAME} src/point_cloud_picture.cpp)
target_link_libraries(${PROJECT_NAME}
   ${OpenCV_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ads_msgs_gencpp)

