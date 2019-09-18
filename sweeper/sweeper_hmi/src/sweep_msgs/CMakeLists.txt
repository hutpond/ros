cmake_minimum_required(VERSION 2.8.3)
project(sweep_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  geometry_msgs
)

add_message_files(DIRECTORY msg FILES
  Light.msg
  BrakeCmd.msg
  BrakeReport.msg
  SystemMode.msg
  FuelLevelReport.msg
  SteeringCmd.msg
  SteeringReport.msg
  ThrottleCmd.msg
  ThrottleReport.msg
  WheelPositionReport.msg
  Sweep.msg
)

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)

#install(DIRECTORY bmr
#        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
#        FILES_MATCHING PATTERN "*.bmr"
#)
