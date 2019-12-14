#include <iostream>

#include <ros/ros.h>
#include "speech_appliction.h"
#include "xtts_offline.h"

int main(int argc, char **argv)
{
  if (login() == 0) {
    std::cout << "tts login failed!" << std::endl;
    return 0;
  }
  ros::init(argc, argv, "path_editor_subscribe_node_");

  SpeechAppliction app;

  while (true) {
    char c = std::cin.get();
    if (c == 'q') {
      break;
    }
    sleep(100 * 1000);
  }
  app.stopSubscribe();
  logout();

  return 0;
}
