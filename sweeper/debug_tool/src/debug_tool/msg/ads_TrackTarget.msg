int64      TRACK_ID
float64    X
float64    Y
float32    SX                                  # target speed 
float32    ANGLE
float32    W
float32    L
float32    H
int8       DEVICE_SOURCE                       #0-UNKNOW 1-RADAR 2-LIDAR 3-CAMERA 5-ZED
int8       MOTION_STATUS                       #preserved
int8       TARGET_TYPE                         #UNKNOW = -1,PEDESTRIAN = 0,BIKE = 1,VEHICLE = 2,MOTORCYCLE=3,BUS=5,	TRUCK = 7 ,TRAFICLIGHT=9,GARBAGE=10
geometry_msgs/Point32[] edge_points
float32    DIRX                                #target x direction
float32    DIRY                                #target y direction 

