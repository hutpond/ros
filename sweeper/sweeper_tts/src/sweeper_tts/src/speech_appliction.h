#ifndef SPEECH_APPLICTION_H
#define SPEECH_APPLICTION_H

#include <memory>
#include <atomic>
#include <thread>
#include <ros/ros.h>
#include <std_msgs/String.h>

class SpeechAppliction
{
public:
  SpeechAppliction();
  void stopSubscribe();

protected:
  void onSubscribe(const std_msgs::String::ConstPtr &);

private:
  std::unique_ptr<ros::NodeHandle> m_nodeHandle;
  ros::Subscriber m_subscirber;

  std::unique_ptr<std::thread> m_pThread;
  std::atomic_bool m_bFlagRun;
};

#endif // SPEECH_APPLICTION_H
