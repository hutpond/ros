#include <sstream>
#include <fstream>
#include <thread>
#include <boost/filesystem.hpp>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "jsoncpp/json/json.h"

void send_planning_string(const ros::Publisher &);

static bool flag_run = true;
int main(int argc, char **argv)
{
  ros::init(argc, argv, "debug_tool_perception_data_send_node");
  ros::NodeHandle n;
  ros::Publisher data_pub = n.advertise<std_msgs::String>("debug_tool_perception_data_send_data", 1000);

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
  const std::string path = "/home/lz/work/doc/sweeper/PerceptionJsonData/";
  while (ros::ok() && flag_run) {
    for (int i = 0; i < 1294; ++i) {
      Json::Reader reader;
      Json::Value root, data;
      root["TYPE"] = "PUSH_UPDATE";

      std::string gps_name = path + std::to_string(i) + "_gps.txt";
      std::ifstream in_gps(gps_name.c_str());
      std::string gps_string((std::istreambuf_iterator<char>(in_gps)),
                          std::istreambuf_iterator<char>());
      in_gps.close();
      Json::Value gps;
      if (!reader.parse(gps_string, gps)) {
        continue;
      }
      data["GPS"] = gps;

      std::string sense_name = path + std::to_string(i) + "_sense.txt";
      std::ifstream in_sense(sense_name.c_str());
      std::string sense_string((std::istreambuf_iterator<char>(in_sense)),
                          std::istreambuf_iterator<char>());
      in_sense.close();
      Json::Value sense, origin_sense;
      if (!reader.parse(sense_string, sense)) {
        continue;
      }
      origin_sense = sense;
      data["ORIGINAL_SENSE"] = origin_sense;
      data["SENSE"] = sense;
      root["DATA"] = data;

      Json::StreamWriterBuilder builder;
      builder["indentation"] = "";
      std::string string_msg = Json::writeString(builder, root);

      std_msgs::String msg;
      msg.data = string_msg;
      data_pub.publish(msg);

      ros::spinOnce();

      usleep(500 * 1000);
      if (!flag_run) break;
    }
    usleep(1000);
  }
}


