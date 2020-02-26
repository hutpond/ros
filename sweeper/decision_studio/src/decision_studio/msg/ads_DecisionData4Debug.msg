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

ads_TrackTarget[] fusion_results

float64 safe_stop_dis
float64 wait_dis

ads_ReferencePoint[] reference_points
ads_Spline[] reference_splines

uint8 Unknown = 0
uint8 Forward = 1               # 前进
uint8 Left_Pass	= 2	            # 左侧绕行
uint8 Right_Pass = 3	        # 右侧绕行
uint8 Follow = 4                # 跟车
uint8 Wait = 5                  # 停车等待
uint8 SafeStop = 6              # 安全停车
uint8 Exit = 7                  # 退出

uint8 decision

string debug_info