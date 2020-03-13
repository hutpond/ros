#include "qcloudpoints.h"

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

#include "QOpenDriveObject.h"
#include "gps.h"

QCloudPoints & QCloudPoints::instance()
{
  static QCloudPoints points;
  return points;
}

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
  for (size_t i = 0; i < size_points; ++i) {
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

void QCloudPoints::openOpenDriverFile(const QString &file_name)
{
  QOpenDriveObject obj;
  QList<QSharedPointer<MapPoint>> reference, road_side;
  obj.readOpenDriveFile(file_name, reference, road_side);

  reference_.clear();
  for (const auto &point : reference) {
    if (reference_.size() == 0) {
      reference_.push_back(point);
    }
    else if (reference_[reference_.size() - 1]->distance(*point) >= 0.5) {
      reference_.push_back(point);
    }
  }

  for (auto point : reference_) {
     this->calcEnuFromLla(point);
  }

  emit loadFinished();
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr QCloudPoints::points() const
{
  return cloud_points_;
}

void QCloudPoints::setOrigin(const Point &point)
{
  origin_point_ = point;
}

void QCloudPoints::setHdMap(const HdMapRaw &hdmap)
{
  hdmap_ = hdmap;
  emit updateData();
}

const HdMapRaw & QCloudPoints::hdMap() const
{
  return hdmap_;
}

void QCloudPoints::clear()
{
  cloud_points_->clear();
  reference_.clear();
  hdmap_.road_segments.clear();
  hdmap_.traffic_lights.clear();
  hdmap_.stop_lines.clear();
  hdmap_.signs.clear();
  hdmap_.markings.clear();
  hdmap_.crossings.clear();
}

void QCloudPoints::calcLlaFromEnu(QSharedPointer<MapPoint> point)
{
  GpsTran gps_tran(origin_point_.x, origin_point_.y, origin_point_.z);

  GpsDataType gps;
  NedDataType ned;

  ned.y_east = point->east;
  ned.x_north = point->north;
  ned.z_down = - point->up;

  gps_tran.fromNedToGps(gps, ned);

  point->lat = gps.latitude;
  point->lon = gps.longitude;
//  point->alt = gps.altitude;
}

void QCloudPoints::calcEnuFromLla(QSharedPointer<MapPoint> point)
{
  GpsTran gps_tran(origin_point_.x, origin_point_.y, origin_point_.z);

  GpsDataType gps;
  NedDataType ned;
  gps.longitude = point->lon;
  gps.latitude  = point->lat;
  gps.altitude  = point->alt;
  gps_tran.fromGpsToNed(ned, gps);

  point->east = ned.y_east;
  point->north = ned.x_north;
//  point->up = -ned.z_down;
}
