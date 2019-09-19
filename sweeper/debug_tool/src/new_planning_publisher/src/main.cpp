#include <iostream>
#include <thread>
#include "ros/ros.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"


void publish_msg();
bool flag_run = false;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "sweeper_debug_planning_publisher_node");

  flag_run = true;
  std::thread thread(publish_msg);
  while (true)
  {
      usleep(100 * 1000);
      char c;
      std::cin.get(c);
      if (c == 'q') break;
  }
  flag_run = false;
  thread.join();

  return 0;
}

void publish_msg()
{
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<debug_ads_msgs::ads_msgs_planning_debug_frame>(
        "planning_debug_data_new", 5);
  ros::Rate loop_rate(100);
  debug_ads_msgs::ads_msgs_planning_debug_frame frame;
  while (ros::ok() && flag_run)
  {
    pub.publish(frame);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
