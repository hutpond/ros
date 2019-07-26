cmake_minimum_required(VERSION 3.0)
string(REGEX REPLACE "/ros$" "" PRE_PATH_STR $ENV{ROS_ROOT})
include(${PRE_PATH_STR}/catkin/cmake/toplevel.cmake)
