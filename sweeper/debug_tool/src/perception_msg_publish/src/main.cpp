#include <unistd.h>
#include <iostream>
#include "ros/ros.h"
#include "application.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_tool_perception_data_publish");
  Application app;
  app.start();

  while (true) {
    char ch;
    std::cin.get(ch);
    if (ch == 'q') break;
    usleep(1000);
  }

  app.stop();
  return 0;
}
