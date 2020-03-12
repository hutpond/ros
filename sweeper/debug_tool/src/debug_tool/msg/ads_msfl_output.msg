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

uint64  unix_ms
float32 pose00
float32 pose01
float32 pose02
float32 pose03
float32 pose10
float32 pose11
float32 pose12
float32 pose13
float32 pose20
float32 pose21
float32 pose22
float32 pose23
float32 pose30
float32 pose31
float32 pose32
float32 pose33


int8  navState      ### 组合导航状态
int8  gpsState      ###GPS状态