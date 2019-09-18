Header  header  
float32 steering_wheel_angle_cmd        #单位degree
int16   steering_wheel_angle_velocity   #单位rad/s, range 0 to 8.7, 0 = maximum
uint8   enable                          #Enable
int8    mode                            #0：机械模式；0x10：助力模式；0x20：角度控制模式；0x22：角度控制模式（解除手动干涉后的角度模式禁止）
int8    middle                          #angular zero