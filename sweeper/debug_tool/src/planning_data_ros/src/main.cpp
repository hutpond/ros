#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "planning_data_ros/PlanningData4Debug.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_tool_planning_data_publish_test");
  ros::NodeHandle node_handle_;
  ros::Publisher planning_pub_ =
      node_handle_.advertise<planning_data_ros::PlanningData4Debug>(
        "planning_debug_data", 1000
        );

  planning_data_ros::PlanningData4Debug data;
  data.vehicle_latitude = 31.815672025974663;
  data.vehicle_longitude = 120.01310435246468;
  data.vehicle_altitude = 10.162395047244551;
  data.vehicle_x = 0;
  data.vehicle_y = 0;
  data.vehicle_z = 0;
  data.vehicle_s = 0;
  data.vehicle_l = 0;
  data.vehicle_width = 1.2;
  data.vehicle_length = 2.5;

  data.safe_dis1 = 2.5;
  data.safe_dis2 = 6.0;
  data.max_planning_distance = 10;
  data.left_half_road_width = 3.5;
  data.right_half_road_width = 4.0;

  data.num_reference_points = 100;
  for (int i = 0; i < 100; ++i) {
    data.reference_points[i].l = 0;
    data.reference_points[i].s = i * 0.15;
    data.reference_points[i].x = data.reference_points[i].s - 0.001 * i;
    data.reference_points[i].y = 2 - 0.01 * i;
  }

  data.debug_info  = "ALTITUDE : 11.4192,"
                     "ENU_X : 0.070064503468351358,"
                     "ENU_Y : 14.39304839022574,"
                     "ENU_Z : -1.6303657624838763e-05,"
                     "L: -0.67181738274531388,"
                     "LATITUDE : 11.4192,"
                     "LONGITUDE : 120.0130952,"
                     "S : 0.23844132023364772,"
                     "VEHICLE_LENGTH : 0,"
                     "VEHICLE_WIDTH : 0";

  while (true) {
    planning_pub_.publish(data);
    ros::spinOnce();
    usleep(1000000);
  }

  return 0;
}
