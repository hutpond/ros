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
  begin_point_display_ = begin_point_;
  end_point_display_ = end_point_;
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

void QCloudPoints::setPointDisplay(pcl::PointXYZ begin, pcl::PointXYZ end)
{
  begin_point_display_ = begin;
  end_point_display_ = end;
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

void QCloudPoints::setSelectedItem(int id)
{
  hdmap_.id_selected = id;
  emit updateData();
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
  point->alt = gps.altitude;
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
  point->up = -ned.z_down;
}

void QCloudPoints::calcRoadSide(double left_w, double right_w, Road *road)
{
  int index_start = 0;
  int index_end = 0;
  bool selected = false;
  for (auto &segment : hdmap_.road_segments) {
    for (auto &road_item : segment.roads) {
      if (hdmap_.id_selected == road_item.second.id) {
        index_start = segment.central_start_index;
        index_end = segment.central_end_index;
        selected = true;
      }
    }
  }
  if (!selected) {
    return;
  }

  auto &left_side = road->left_side;
  auto &right_side = road->right_side;
  left_side.clear();
  right_side.clear();

  for (int i = index_start; i <= index_end; ++i) {
    QPointF ptf = QPointF(reference_[i]->east, reference_[i]->north);
    QLineF linef, linef_left, linef_right;
    if (i == index_start) {
      linef = QLineF(reference_[i]->east, reference_[i]->north,
          reference_[i + 1]->east, reference_[i + 1]->north);
    }
    else if (i == index_end) {
      linef = QLineF(reference_[i - 1]->east, reference_[i - 1]->north,
          reference_[i]->east, reference_[i]->north);
    }
    else {
      linef = QLineF(reference_[i - 1]->east, reference_[i - 1]->north,
          reference_[i + 1]->east, reference_[i + 1]->north);
    }
    linef_left = QLineF(ptf, ptf + QPointF(1, 0));
    linef_right = QLineF(ptf, ptf + QPointF(1, 0));

    double angle = linef.normalVector().angle();
    linef_left.setLength(left_w);
    linef_left.setAngle(180 + angle);
    QSharedPointer<MapPoint> point_left(new MapPoint);
    point_left->id = reference_[i]->id;
    point_left->east = linef_left.p2().x();
    point_left->north = linef_left.p2().y();
    point_left->up = reference_[i]->up;
    this->calcLlaFromEnu(point_left);
    Point point;
    point.x = point_left->east;
    point.y = point_left->north;
    point.z = point_left->up;
    left_side.push_back(point);

    linef_right.setLength(right_w);
    linef_right.setAngle(angle);
    QSharedPointer<MapPoint> point_right(new MapPoint);
    point_right->id = reference_[i]->id;
    point_right->east = linef_right.p2().x();
    point_right->north = linef_right.p2().y();
    point_right->up = reference_[i]->up;
    this->calcLlaFromEnu(point_right);
    point.x = point_right->east;
    point.y = point_right->north;
    point.z = point_right->up;
    right_side.push_back(point);
  }
}
