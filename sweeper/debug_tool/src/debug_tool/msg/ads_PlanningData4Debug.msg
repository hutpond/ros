Header header

float64 vehicle_latitude
float64 vehicle_longitude
float64 vehicle_altitude
float64 vehicle_x
float64 vehicle_y
float64 vehicle_z
float64 vehicle_s
float64 vehicle_l
float64 vehicle_width
float64 vehicle_length
float64 head_distance

ads_RadarTarget[] radar28f_results
ads_RadarTarget[] radar73f_results
ads_UltraSonicTarget[] ultrasonic_results
ads_TrackTarget[] fusion_results
ads_garbage_detection[] garbage_detection_results

float64 safety_cost_weight
float64 lateral_cost_weight
float64 smoothness_cost_weight
float64 consistency_cost_weight
float64 garbage_cost_weight
ads_planning_trajectory[] planning_trajectory_candidates
ads_planning_trajectory planning_trajectory
ads_planning_output planning_output

float64 safe_dis1
float64 safe_dis2
float64 max_planning_distance
ads_pubcurb curb
bool left_road_boundary_available
bool right_road_boundary_available
ads_Spline[] left_road_boundary_splines
ads_Spline[] right_road_boundary_splines
float64 left_road_boundary_start_s
float64 left_road_boundary_end_s
float64 right_road_boundary_start_s
float64 right_road_boundary_end_s
float64 left_road_width
float64 right_road_width

ads_ReferencePoint[] reference_points
ads_Spline[] reference_splines

uint8 decision
uint8 ultrasonic_decision
uint8 radar28f_decision
uint8 radar73f_decision
uint8 track_target_decision
ads_TargetPoint[4] decision_targets

string debug_info