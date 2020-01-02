#ifndef POINT_CLOUD_PICTURE_H
#define POINT_CLOUD_PICTURE_H

#include <functional>

class PointCloudPicturePrivate;
class PointCloudPicture
{
public:
  PointCloudPicture();
  ~PointCloudPicture();

  void startSubscirbe();
  void stopSubscirbe();
  void setSendFunction(std::function<void(const std::string &)>);
  void setIntervalMSecond(int);

private:
  PointCloudPicturePrivate *picture_private_ptr_;
};

#endif // POINT_CLOUD_PICTURE_H
