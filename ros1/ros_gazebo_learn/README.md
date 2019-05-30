command list
=====================

roslaunch urdf_tutorial display.launch model:=urdf/robot.urdf.xacro
roslaunch urdf_sim_tutorial gazebo.launch model:=urdf/robot.urdf.xacro
rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[1.0,0,0]' '[0,0,0.0]'


rostopic pub -r 30 /mobile_base_controller/cmd_vel geometry_msgs/Twist -- '[1.0,0,0]' '[0,0,0.0]'
