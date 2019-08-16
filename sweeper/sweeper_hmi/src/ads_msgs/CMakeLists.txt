cmake_minimum_required(VERSION 2.8.3)
project(ads_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs sensor_msgs jsk_recognition_msgs)

add_message_files(
  DIRECTORY msg

  FILES
  ads_ins_data.msg

  ads_module_report.msg

  ads_ad_command.msg
  ads_ad_report.msg

  ads_planning_output.msg
  ads_ReferencePoint.msg
  ads_TargetPoint.msg
  ads_PlanningData4Debug.msg

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
  
  ads_msfl_output.msg

  ads_boxInfo.msg
  ads_CameraCIPX.msg
  ads_CameraCurb.msg
  ads_CameraCurbsLanes.msg
  ads_CameraLane.msg
  ads_CameraPedTarget.msg
  ads_CameraTargetColl.msg
  ads_CameraVehTarget.msg
  ads_obsBoxes.msg
  ads_pubcurb.msg
  ads_Radar28fTargetColl.msg
  ads_Radar73fTargetColl.msg
  ads_RadarTarget.msg
  ads_TrackTarget.msg
  ads_TrackTargetColl.msg
  ads_TrackTargetCollDbg.msg
  ads_UltraSonicTarget.msg
  ads_UltraSonicTargetColl.msg
  ads_VehicleLocation.msg

  ads_lidar_front.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  jsk_recognition_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs jsk_recognition_msgs sensor_msgs
)
