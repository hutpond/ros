Header header

float64 vehicle_enu_x
float64 vehicle_enu_y
float64 vehicle_enu_z
float64 vehicle_pitch
float64 vehicle_roll
float64 vehicle_yaw
ads_Point front_axle_center
ads_Point rear_axle_center
ads_Point head_point
ads_Point rear_point
ads_Point hinge_point
float64 front_vehicle_length
float64 front_vehicle_width
float64 rear_vehicle_length
float64 rear_vehicle_width
float64 steering_angle

ads_Point[] reference_points
ads_Spline[] reference_splines

ads_Point radar_Point
ads_RadarTarget[] radar_results
ads_Point[14] ultrasonic_points
ads_UltraSonicTarget[] ultrasonic_results
ads_TrackTarget[] fusion_results
ads_garbage_detection[] garbage_detection_results

uint8 Unknown = 0
uint8 Forward = 1										# 前进
uint8 LeftPass = 2										# 左侧绕行
uint8 RightPass = 3										# 右侧绕行
uint8 Follow = 4										# 跟随
uint8 Wait = 5                                          # 停车等待
uint8 SafeStop = 6										# 安全停车
uint8 Exit = 7											# 退出

uint8 decision
uint8 radar_decision
uint8 ultrasonic_decision
uint8 track_target_decision

float64 safety_cost_weight
float64 lateral_cost_weight
float64 smoothness_cost_weight
float64 consistency_cost_weight
float64 garbage_cost_weight
ads_planning_trajectory[] planning_trajectory_candidates
ads_planning_trajectory planning_trajectory
ads_planning_output planning_output
uint8 scenario_type
uint8 NORMAL = 0
uint8 YIELD = 1

string debug_info