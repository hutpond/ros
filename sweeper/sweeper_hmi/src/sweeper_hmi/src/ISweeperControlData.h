// Declare key_xxx likely variable and "data_xxx" likely string.

// In C++ module, same as: const std::string key_latitude = "data_latitude"
DECLARE_SWEEPER_CONTROL_DATA(latitude)                  //纬度，degree, ro

DECLARE_SWEEPER_CONTROL_DATA(longitude)                 //经度，degree, ro
DECLARE_SWEEPER_CONTROL_DATA(velocity)                  //时速，m/s, ro

DECLARE_SWEEPER_CONTROL_DATA(yaw_angle)                 //车头指向方向, degree, ro

DECLARE_SWEEPER_CONTROL_DATA(battery_voltage)           //伏, ro
DECLARE_SWEEPER_CONTROL_DATA(battery_temperature)       //ro
DECLARE_SWEEPER_CONTROL_DATA(battery_remaining_capacity)  //电池剩余电量 %,ro
DECLARE_SWEEPER_CONTROL_DATA(battery_capacity)          //ro

DECLARE_SWEEPER_CONTROL_DATA(light)                     //工作灯, 0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(reverse_light)             //倒车灯，0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(brake_light)               //刹车灯，0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(left_light)                //左转向灯状态,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(right_light)               //右转向灯状态,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(high_beam_light)           //远光灯,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(low_beam_light)            //近光灯,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(width_light)               //示廓灯,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(emergency_light)           //应急灯,0:关，1：开 , rw

DECLARE_SWEEPER_CONTROL_DATA(total_mileage)             //自动驾驶总里程
DECLARE_SWEEPER_CONTROL_DATA(total_time)                //自动驾驶总时间

DECLARE_SWEEPER_CONTROL_DATA(gear)                      //档位:"N","P","D","R", ro
DECLARE_SWEEPER_CONTROL_DATA(steer_angle_value)         //当前方向盘转角:度, ro
DECLARE_SWEEPER_CONTROL_DATA(throttle_value)            //油门值, ro
DECLARE_SWEEPER_CONTROL_DATA(brake_value)               //刹车值：0-100%, ro, 当前制动踏板角度
DECLARE_SWEEPER_CONTROL_DATA(manual_brake)              //手刹状态 ,0:关，1：开, ro

DECLARE_SWEEPER_CONTROL_DATA(water_level)               //清水箱液位: 0-100%, ro
DECLARE_SWEEPER_CONTROL_DATA(trash_level)               //垃圾箱内垃圾位 , ro
DECLARE_SWEEPER_CONTROL_DATA(trash_position)            //垃圾箱位置, 1：落下完成，2：抬起完成，0,3：抬起或落下未完成， ro
DECLARE_SWEEPER_CONTROL_DATA(brush_position)            //扫刷位置 ,0:上，1：下, rw
DECLARE_SWEEPER_CONTROL_DATA(brush_status)              //扫刷状态 ,0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(suction_status)            //吸风状态, 0:关，1：开, rw
DECLARE_SWEEPER_CONTROL_DATA(suction_inlet_position)    //吸风口位置 ,0:上，1：下, rw
DECLARE_SWEEPER_CONTROL_DATA(spout_water)               //喷水状态,0:关，1：开, rw

//value format: "101"
DECLARE_SWEEPER_CONTROL_DATA(lidar_status)              //激光雷达 多路, ro, 每个字符代表一个雷达状态， '0':有问题，'1'：正常, '2':N/A， 字符串长度代表有多少个雷达

//value format: "10110010"
DECLARE_SWEEPER_CONTROL_DATA(ultrasonic_wave_status)    //多路 超声波雷达, ro, 每个字符代表一个雷达状态， '0':有问题，'1'：正常, '2':N/A， 字符串长度代表有多少个雷达

//value format: "01"
DECLARE_SWEEPER_CONTROL_DATA(millimeter_wave_status)    //多路 毫米波雷达, ro, 每个字符代表一个雷达状态， '0':有问题，'1'：正常, '2':N/A， 字符串长度代表有多少个雷达

//value format: "10"
DECLARE_SWEEPER_CONTROL_DATA(zed_status)                //多路 深度相机, ro, 每个字符代表一个lidar的状态， '0':有问题，'1'：正常, '2':N/A， 字符串长度代表有多少个雷达

DECLARE_SWEEPER_CONTROL_DATA(rtk_status)                //RTK信号强度(-1：offline， 0-100), ro

DECLARE_SWEEPER_CONTROL_DATA(gps_signal)                //GPS状态信息(-1：offline， 0-100), ro
DECLARE_SWEEPER_CONTROL_DATA(wifi_signal)               //(-1：offline， 0-100), ro
DECLARE_SWEEPER_CONTROL_DATA(telcom_signal)             //4g/5g状态信息(-1：offline， 0-100), ro
DECLARE_SWEEPER_CONTROL_DATA(dtu_signal)                //dtu状态信息(-1：offline， 0-100), ro

DECLARE_SWEEPER_CONTROL_DATA(software_status)           //自动驾驶算法加载状态： 0:有问题，1：正常, ro

DECLARE_SWEEPER_CONTROL_DATA(sweeper_state)             //系统状态： 0：待机， 1：自动作业中， 2：手动驾驶中，  3： 故障, ro

DECLARE_SWEEPER_CONTROL_DATA(software_version)          //软件版本， ro
DECLARE_SWEEPER_CONTROL_DATA(algorithm_version)         //算法版本， ro
DECLARE_SWEEPER_CONTROL_DATA(vcu_version)               //VCU版本： -1:N/A, ro 
DECLARE_SWEEPER_CONTROL_DATA(imu_status)                //IMU状态：0:有问题，1：正常, ro

DECLARE_SWEEPER_CONTROL_DATA(fault_list)                //车故障列表，以\n分隔的字符串:"1-1-1\n2-1-1...", ro

DECLARE_SWEEPER_CONTROL_DATA(planning_data_debug)       //debug tool data, ro

//Use by internal
DECLARE_SWEEPER_CONTROL_DATA(car_state)                 //车当前状态，ro
DECLARE_SWEEPER_CONTROL_DATA(module_state_changed)      //软件模块状态改变，1: 已经改变， ro
