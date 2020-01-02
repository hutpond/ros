int8    GPSSTATE_NONE = 0           #启动失败
int8    GPSSTATE_SINGLEPOINT = 1    #单点
int8    GPSSTATE_FLOATPOINT = 2     #浮点  
int8    GPSSTATE_RTK = 3            #RTK
int8    GPSSTATE_DRONLY = 4         #dead reckoning only
int8    GPSSTATE_TIMEONLY = 5       #time only fix
int8    GPSSTATE_GNSS_DR = 6        #GNSS + dead reckoning combined

Header  header
int8    head                        # 帧头
uint64  gps_ms                      # GPS时间: 周内秒，单位ms
float64 lat                         # degree
float64 lon                         # degree
float64 height                      # m
float64 velEast                     # m/s
float64 velNorth                    # m/s
float64 velUp                       # m/s
float64 pitch                       # degree, 向上为正,   [-90,  90]
float64 roll                        # degree, 右倾为正,    [-180, 180]
float64 yaw                         # degree, 北偏东为正,   [0,  360]
float64 accX                        # g
float64 accY                        # g
float64 accZ                        # g
float64 gyroX                       # degree/s
float64 gyroY                       # degree/s
float64 gyroZ                       # degree/s
int8    gpsState                    # 定位状态 gpsState_xxx
int8    updateFlag                  # 数据更新标记，1：已更新；0：未更新
int8    solnState                   #
int8    solnSVs                     #卫星个数
int8    anglesPosType               #    
float64 heading_GNSS                #
int8    wheelspeed_state            # 轮速有无标志
#New field
int8    locationIsAligned           #1 表示位置完成初对准
int8    velcoityIsAligned           #1 表示速度完成初对准
int8    poseIsAligned               #1 表示姿态完成初对准
int8    yawIsAligned                #1 表示航向角完成初对准
int8    gpsOrientState              # 定向状态 gpsState_xxx
