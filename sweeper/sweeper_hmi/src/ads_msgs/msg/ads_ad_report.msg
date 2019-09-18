uint8 carState_Idle = 0

#Auto-drive
uint8 carState_Moving = 1
uint8 carState_MoveDone = 2
uint8 carState_Sweeping = 3
uint8 carState_SweepDone = 4
uint8 carState_Stop = 5

#Manual drive
uint8 carState_Manual = 6

#
uint8 carState_Busy = 7

Header    header

uint8     carState
float64   carSpeed
float64   carMileage
