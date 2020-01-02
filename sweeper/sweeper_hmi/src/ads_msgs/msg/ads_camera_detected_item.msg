int8 typeID_UNKNOW         = -1
int8 typeID_PEDESTRIAN     = 0
int8 typeID_BIKE           = 1
int8 typeID_VEHICLE        = 2
int8 typeID_MOTORCYCLE     = 3
int8 typeID_BUS            = 5
int8 typeID_TRUCK          = 7
int8 typeID_TRAFICLIGHT    = 9
int8 typeID_GARBAGE        = 10

int8                    typeID
uint32                  trackID
float64                 width               #鸟瞰图障碍物宽度
float64                 height              #鸟瞰图障碍物高度=0
float64                 length              #鸟瞰图障碍物长度
geometry_msgs/Point     center              #鸟瞰图，车体坐标系下，障碍物中心点坐标
geometry_msgs/Point[]   points              #鸟瞰图，车体坐标系下，障碍物角点坐标
geometry_msgs/Point[]   outline             #这两个点是示宽，这两个点的距离就是被检测物体现实世界中的宽度
