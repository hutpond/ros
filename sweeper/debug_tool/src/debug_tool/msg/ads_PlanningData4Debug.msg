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

ads_Radar28fTargetColl radar28f_results
ads_Radar73fTargetColl radar73f_results
ads_UltraSonicTargetColl ultrasonic_results
ads_TrackTargetColl fusion_results
ads_littersensor_data garbage_detection_results

uint8 num_planning_splines
ads_Spline[100] planning_splines
ads_planning_output planning_output

float64 safe_dis1
float64 safe_dis2
float64 max_planning_distance
ads_pubcurb curb
bool left_road_boundary_available
bool right_road_boundary_available
float64[5] left_road_boundary
float64[5] right_road_boundary
uint8 num_left_road_boundary_splines
ads_Spline[100] left_road_boundary_splines
uint8 num_right_road_boundary_splines
ads_Spline[100] right_road_boundary_splines
float64 left_road_boundary_start_s
float64 left_road_boundary_end_s
float64 right_road_boundary_start_s
float64 right_road_boundary_end_s
float64 left_half_road_width
float64 right_half_road_width

uint8 num_reference_points
ads_ReferencePoint[100] reference_points
uint8 num_reference_splines
ads_Spline[100] reference_splines

uint8 decision
uint8 ultrasonic_decision
uint8 radar28f_decision
uint8 radar73f_decision
ads_TargetPoint[6] decision_targets

string debug_info