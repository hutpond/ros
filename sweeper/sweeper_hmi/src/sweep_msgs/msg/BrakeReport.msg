Header header

# Brake pedal
# Unitless, range 0.15 to 0.50
float32 pedal_input
float32 pedal_cmd
int16 pedal_output # 0~2500:0~100%(完全制动-完全释放)， 每500一个档位

# Braking torque (Nm)
float32 torque_input
float32 torque_cmd
float32 torque_output


bool emergency_status # 急停按钮状态　
bool boo_cmd
bool boo_output

# Status
bool enabled  # Enabled
bool override # Driver override
bool driver   # Driver activity
bool timeout  # Command timeout



