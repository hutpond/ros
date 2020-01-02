#ifndef POINT_CLOUD_PICTURE_PRIVATE_H
#define POINT_CLOUD_PICTURE_PRIVATE_H

#include <thread>
#include <atomic>

#include "ros/node_handle.h"
#include "sensor_msgs/PointCloud2.h"
#include "ads_msgs/ads_TrackTargetColl.h"

class PointCloudPicturePrivate
{
public:
  PointCloudPicturePrivate();

protected:
  void onSubscirbePointCloud(const sensor_msgs::PointCloud2 &);
  void onSubscirbeTargets(const ads_msgs::ads_TrackTargetColl &);
  void run();

  void createImage();

private:
  ros::NodeHandle node_handle_;
  ros::Subscriber subscriber_point_;
  ros::Subscriber subscriber_targets_;

  std::shared_ptr<std::thread> thread_spin_;
  std::atomic_bool flag_subscribe_;
  int interval_ms_;
  std::function<void(const std::string &)> send_function_;

  sensor_msgs::PointCloud2 point_cloud_;
  ads_msgs::ads_TrackTargetColl track_targets_;

  friend class PointCloudPicture;
};

#endif // POINT_CLOUD_PICTURE_PRIVATE_H
