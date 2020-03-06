#include "qcloudpoints.h"

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

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

pcl::PointCloud<pcl::PointXYZ>::ConstPtr QCloudPoints::points() const
{
  return cloud_points_;
}

void QCloudPoints::addRoadSegment(int type)
{
  RoadSegment road_segment;
  road_segment.type = type;
  hdmap_.road_segments.push_back(road_segment);
}

void QCloudPoints::addRoad(int index_segment, int index_road)
{
  if (index_segment < 0 && index_segment >= hdmap_.road_segments.size()) {
    return;
  }

  auto &roads = hdmap_.road_segments[index_segment].roads;
  Road road;
  roads[index_road] = road;
}

void QCloudPoints::addRoadPoint(int index_segment, int index_road, int type, const Point &point)
{
  auto &segments = hdmap_.road_segments;
  if (index_segment < 0 || index_segment >= segments.size()) {
    return;
  }

  auto &roads = segments[index_segment].roads;
  auto road = roads.find(index_road);
  if (road != roads.end()) {
    if (type == Road::LEFT) {
      road->second.left_side.push_back(point);
    }
    else if (type == Road::RIGHT) {
      road->second.right_side.push_back(point);
    }
    else {
      road->second.reference.push_back(point);
    }
  }
}
