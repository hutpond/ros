#include <sstream>
#include <fstream>
#include <thread>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"

void send_planning_string(const ros::Publisher &);

static bool flag_run = true;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_tool_planning_data_send_node");
  ros::NodeHandle n;
  ros::Publisher data_pub = n.advertise<std_msgs::String>("debug_tool_planning_data_send_data", 1000);

  std::thread thread(&send_planning_string, data_pub);

  while (true) {
      usleep(100 * 1000);
      char c;
      std::cin.get(c);
      if (c == 'q') break;
  }
  flag_run = false;
  thread.join();

  return 0;
}

void send_planning_string(const ros::Publisher &data_pub)
{
  namespace fs = boost::filesystem;
  fs::path path = "/home/lz/work/doc/sweeper/JsonDataTest";
  while (ros::ok() && flag_run) {
    for (auto &it : fs::directory_iterator(path)) {
      std::stringstream ss;
      ss << it;
      std::string name = ss.str();

      name = name.substr(1, name.length() - 2);
      std::ifstream in(name.c_str());
      std::string planning_string((std::istreambuf_iterator<char>(in)),
                          std::istreambuf_iterator<char>());
      in.close();

      std_msgs::String msg;
      msg.data = planning_string;
      data_pub.publish(msg);

      ros::spinOnce();

      usleep(500 * 1000);
      if (!flag_run) break;
    }
    usleep(1000);
  }
}


