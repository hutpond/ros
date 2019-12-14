#include "speech_appliction.h"
#include "xtts_offline.h"

SpeechAppliction::SpeechAppliction()
{
  m_nodeHandle.reset(new ros::NodeHandle);
  m_subscirber = m_nodeHandle->subscribe<std_msgs::String>(
        "/sweeper_tts_text",
        5,
        &SpeechAppliction::onSubscribe, this);

  m_bFlagRun = true;
  std::thread *thread = new std::thread([&](){
    while (m_bFlagRun) {
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  });
  m_pThread.reset(thread);
}

void SpeechAppliction::stopSubscribe()
{
  m_bFlagRun = false;
  m_pThread->join();
}

void SpeechAppliction::onSubscribe(const std_msgs::String::ConstPtr &text)
{
  std::cout << text->data << std::endl;

  const char* src_text = text->data.c_str();
  const char* des_path = "/tmp/1.wav";
  const char* param = "";
  int ret = text_to_speech(src_text, des_path, param);
  if (ret == 0) {
    const char *cmd = "ffplay -nodisp -loop 1 -autoexit /tmp/1.wav";
    system(cmd);
  }
}
