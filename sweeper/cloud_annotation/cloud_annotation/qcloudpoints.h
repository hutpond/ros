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
  const QList<QSharedPointer<MapPoint>> & reference() const {return reference_;}

  void setOrigin(const Point &);
  void setHdMap(const HdMapRaw &);
  const HdMapRaw & hdMap() const;
  void clear();

protected:
  void calcLlaFromEnu(QSharedPointer<MapPoint>);
  void calcEnuFromLla(QSharedPointer<MapPoint>);

signals:
  void updateData();
  void loadFinished();

private:
  explicit QCloudPoints(QObject *parent = nullptr);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
  pcl::PointXYZ begin_point_;
  pcl::PointXYZ end_point_;
  QList<QSharedPointer<MapPoint>> reference_;

  Point origin_point_;
  HdMapRaw hdmap_;
};

#endif // QCLOUDPOINTS_H
