int64      TRACK_ID
float64    X
float64    Y
float32    SX
float32    ANGLE
float32    W
float32    L
float32    H
int8       DEVICE_SOURCE                       #0-UNKNOW 1-RADAR 2-LIDAR 3-CAMERA 5-ZED
int8       MOTION_STATUS                       #preserved
int8       TARGET_TYPE                         #UNKNOW = -1,VEHICLE = 0,TRUCK = 1,BIKE = 2,	PEDESTRIAN = 3,MOTORCYCLE =4,GARBAGE=5
geometry_msgs/Point32[] edge_points
