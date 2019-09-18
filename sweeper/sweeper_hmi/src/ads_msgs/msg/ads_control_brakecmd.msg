Header  head    
float32 pedal_cmd           #Brake pedal Options defined below
uint8   pedal_cmd_type  
uint8   enable              #Enable
uint8   clear               #Clear driver overrides
uint8   ignore              #Ignore driver overrides
uint8   count               #Watchdog counter (optional)
uint8   CMD_NONE=0  
uint8   CMD_PEDAL=1         #Unitless, range 0.15 to 0.50
uint8   CMD_PERCENT=2       #Percent of maximum torque, range 0 to 1
uint8   CMD_TORQUE=3        #range 0 to 3412, open-loop，单位Nm
uint8   CMD_TORQUE_RQ=4     #range 0 to 3412, closed-loop，单位Nm
uint8   CMD_DECEL=6         #range 0 to 10，单位m/s^2
float32 TORQUE_BOO=520      #brake lights threshold，单位Nm
float32 TORQUE_MAX=3412     #maximum torque，单位Nm