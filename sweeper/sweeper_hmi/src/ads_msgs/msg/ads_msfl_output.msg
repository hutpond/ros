## data structure of msfl output
int8    head        ### 帧头
uint64  gps_ms      ### GPS时间: 周内秒，单位ms
float64  lat        ### degree
float64  lon        ### degree
float64  height     ### m
float64  velEast    ### m/s
float64  velNorth   ### m/s
float64  velUp      ### m/s
float64  pitch      ### degree, 向上为正,   [-90,  90]
float64  roll       ### degree, 右倾为正,    [-180, 180]
float64  yaw        ### degree, 北偏东为正,   [-180,180] / [0,  360]
float64  accX       ### m/s^2
float64  accY       ### m/s^2
float64  accZ       ### m/s^2
float64  gyroX      ### degree/s
float64  gyroY      ### degree/s
float64  gyroZ      ### degree/s

int8  navState      ### 组合导航状态
int8  gpsState      ###GPS状态
