Header  header  
float32 pedal_input         #Brake pedal Unitless, range 0.15 to 0.50
float32 pedal_cmd   
int16   pedal_output        #0-2500：0-100%（完全制动-完全释放），每500一个档位
float32 torque_input        #Braking torque，单位Nm
float32 torque_cmd  
float32 torque_output   
uint8   emergency_status    #急停按钮状态
uint8   boo_cmd 
uint8   boo_output  
uint8   enabled             #Enabled
uint8   override            #Driver override
uint8   driver              #Driver activity
uint8   timeout             #Command timeout