#ifndef QCLOUDPOINTS_H
#define QCLOUDPOINTS_H

#include <QObject>
#include <QVector>
#include <QSharedPointer>
#include <pcl/io/io.h>

#include "GlobalDefine.h"

class QCloudPoints : public QObject
{
  Q_OBJECT
public:
  static QCloudPoints & instance();

  void openFile(const QString &);
  void openOpenDriverFile(const QString &);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points() const;
  pcl::PointXYZ begin_point() {return begin_point_;}
  pcl::PointXYZ end_point() {return end_point_;}

  void setHdMap(const HdMapRaw &);
  const HdMapRaw & hdMap() const;
  void clear();

signals:
  void updateData();

private:
  explicit QCloudPoints(QObject *parent = nullptr);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
  pcl::PointXYZ begin_point_;
  pcl::PointXYZ end_point_;
  QList<QSharedPointer<MapPoint>> reference_;

  HdMapRaw hdmap_;
};

#endif // QCLOUDPOINTS_H
