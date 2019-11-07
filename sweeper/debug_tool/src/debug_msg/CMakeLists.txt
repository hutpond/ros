cmake_minimum_required(VERSION 2.8.3)
project(debug_ads_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs jsk_recognition_msgs)

add_message_files(
  DIRECTORY msg

  FILES
        ads_msgs_planning_debug_frame.msg
        ads_msgs_planning_debug_obstacle.msg
        ads_msgs_planning_debug_param.msg
        ads_msgs_planning_debug_pointENU.msg
        ads_msgs_planning_debug_pointFRENET.msg
        ads_msgs_planning_debug_state_machine.msg
        ads_msgs_planning_debug_trajectory.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
