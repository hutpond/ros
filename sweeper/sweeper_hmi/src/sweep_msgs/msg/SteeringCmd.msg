Header header
# Steering Wheel
float32 steering_wheel_angle_cmd        # degree
int16 steering_wheel_angle_velocity   # rad/s, range 0 to 8.7, 0 = maximum

# Enable
bool enable

# control mode , 0: 机械模式;0x10:助力模式;0x20:角度控制模式;0x22:角度控制模式(解除手动干涉后的角度模式禁止);
int8 mode

# angular zero
int8 middle




