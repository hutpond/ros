#include "message_publish.h"

MessagePublish::MessagePublish()
{
  track_pub_ = node_handle_.advertise<perception_msg_publish::TrackTargetColl>(
        "perception_debug_data_track_target_coll", 10
        );
  ultrasonic_pub_ = node_handle_.advertise<perception_msg_publish::UltrasonicTargetColl>(
        "perception_debug_data_ultrasonic_target_coll", 10
        );

  msgoutput_pub_ = node_handle_.advertise<perception_msg_publish::msflOutput>(
        "msfl_output", 10
        );
}

void MessagePublish::publish_track_target(const perception_msg_publish::TrackTargetColl &msg)
{
  track_pub_.publish(msg);
  ros::spinOnce();
}

void MessagePublish::publish_ultrasonic_target(
    const perception_msg_publish::UltrasonicTargetColl &msg)
{
  ultrasonic_pub_.publish(msg);
  ros::spinOnce();
}

void MessagePublish::publish_msfl_output(const perception_msg_publish::msflOutput &msg)
{
  msgoutput_pub_.publish(msg);
  ros::spinOnce();
}
