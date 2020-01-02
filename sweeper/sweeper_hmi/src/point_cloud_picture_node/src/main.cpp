#include <iostream>
#include "ros/ros.h"

#include "point_cloud_picture.h"

void send_image(const std::string &);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "POINT_CLOUD_PICTURE_SEND_NODE");

  PointCloudPicture point_cloud;
  point_cloud.setIntervalMSecond(200);
  point_cloud.setSendFunction(send_image);
  point_cloud.startSubscirbe();

  while (true) {
    usleep(100 * 1000);
    char c;
    std::cin.get(c);
    if (c == 'q') break;
  }
  point_cloud.stopSubscirbe();
}

void send_image(const std::string &name)
{
  std::cout << "send " << name << std::endl;
}
