#For topic "/ads_module_report"
#string moduleName_Perception_    = "感知模块"
#string moduleName_Localization   = "定位模块"
#string moduleName_Planning       = "规划模块"
#string moduleName_Control        = "控制模块"
#string moduleName_Software       = "软件"
#string moduleName_MCU            = "MCU"
#string moduleName_EPS            = "EPS"
#string moduleName_BMS            = "BMS"

#(moduleStatus==moduleStatus_OK && moduleSubtype==0) means moduleID works well, no issue.
# All previous reported issue will be clear.

# Every module will publish "moduleStatus_Info_Online" periodically.

uint8 moduleID_Perception_    = 1
uint8 moduleID_Localization   = 2
uint8 moduleID_Planning       = 3
uint8 moduleID_Control        = 4
uint8 moduleID_Software       = 5
uint8 moduleID_MCU            = 6
uint8 moduleID_EPS            = 7
uint8 moduleID_BMS            = 8

uint8 moduleStatus_OK           = 0
uint8 moduleStatus_Critical     = 1
uint8 moduleStatus_Warning      = 2
uint8 moduleStatus_Info_Online  = 3

uint8     moduleID
uint8     moduleStatus
uint8     moduleSubtype
uint8     deviceID
