cmake_minimum_required(VERSION 2.8.3)
project(ads_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg

  FILES
  ads_lidar_data.msg
  ads_ins_data.msg

  ads_device_report.msg     
  ads_module_report.msg

  ads_planningv1_status.msg
  ads_planningv1_trajectory.msg
  ads_planningv2_pathpoint.msg
  ads_planningv2_trajectory.msg
  ads_planningv2_decision.msg

  ads_control_brakecmd.msg
  ads_control_brakereport.msg
  ads_control_fuel_level_report.msg
  ads_control_light.msg
  ads_control_steeringcmd.msg
  ads_control_steering_report.msg
  ads_control_sweep.msg
  ads_control_systemmode.msg
  ads_control_throttlecmd.msg
  ads_control_throttle_report.msg
  ads_control_WheelPositionReport.msg
  
  ads_garbage_detection.msg
  ads_littersensor_data.msg

  ads_hmi_command.msg  
  
  ads_msfl_output.msg

)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
