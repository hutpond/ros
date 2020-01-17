Header  header              # 帧头
uint8   decision            # 决策状态
geometry_msgs/Pose  pose    # 规划点位姿
float32 velocity            # 期望速度

uint8   Forward = 0         # 跟随默认轨迹
uint8   Follow = 1          # 跟随同方向障碍物
uint8   LeftPass = 2        # 向左绕行
uint8   RightPass = 3       # 向右绕行
uint8   SafeStop = 4        # 安全停车