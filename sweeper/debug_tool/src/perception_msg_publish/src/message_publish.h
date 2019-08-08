#ifndef MESSAGE_PUBLISH_H
#define MESSAGE_PUBLISH_H

#include "ros/ros.h"
#include "perception_msg_publish/TrackTargetColl.h"
#include "perception_msg_publish/TrackTarget.h"
#include "perception_msg_publish/UltrasonicTarget.h"
#include "perception_msg_publish/UltrasonicTargetColl.h"
#include "perception_msg_publish/msflOutput.h"

class MessagePublish
{
public:
  MessagePublish();

  void publish_track_target(const perception_msg_publish::TrackTargetColl &);
  void publish_ultrasonic_target(const perception_msg_publish::UltrasonicTargetColl &);
  void publish_msfl_output(const perception_msg_publish::msflOutput &);

private:
  ros::NodeHandle node_handle_;
  ros::Publisher track_pub_;
  ros::Publisher ultrasonic_pub_;
  ros::Publisher msgoutput_pub_;
};

#endif // MESSAGE_PUBLISH_H
