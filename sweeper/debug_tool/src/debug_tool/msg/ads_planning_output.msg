Header  header              # 帧头
uint8   decision            # 决策状态
geometry_msgs/Pose  pose    # 规划点位姿
float32 velocity            # 期望速度

uint8   Forward = 0         # 前进
uint8   Follow = 1          # 跟随
uint8   LeftPass = 2        # 左侧绕行
uint8   RightPass = 3       # 右侧绕行
uint8   SafeStop = 4        # 安全停车
uint8   Exit = 5            # 退出自动驾驶