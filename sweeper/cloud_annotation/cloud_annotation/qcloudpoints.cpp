#include "qcloudpoints.h"

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

QCloudPoints::QCloudPoints(QObject *parent) : QObject(parent)
{
  cloud_points_.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void QCloudPoints::openFile(const QString &fileName)
{
  cloud_points_->clear();
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toLocal8Bit().data(), *cloud_points_) == -1) {
    return;
  }

  begin_point_ = pcl::PointXYZ(1000, 1000, 1000);
  end_point_ = pcl::PointXYZ(-1000, -1000, -1000);
  const size_t size_points = cloud_points_->size();
  select_flag_.resize(size_points);
  for (size_t i = 0; i < size_points; ++i) {
    select_flag_[i].reset(new int);
    *select_flag_[i] = 0;
    auto &point = cloud_points_->at(i);

    if (begin_point_.x > point.x) {
      begin_point_.x = point.x;
    }
    if (begin_point_.y > point.y) {
      begin_point_.y = point.y;
    }
    if (begin_point_.z > point.z) {
      begin_point_.z = point.z;
    }

    if (end_point_.x < point.x) {
      end_point_.x = point.x;
    }
    if (end_point_.y < point.y) {
      end_point_.y = point.y;
    }
    if (end_point_.z < point.z) {
      end_point_.z = point.z;
    }
  }
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr QCloudPoints::points() const
{
  return cloud_points_;
}
