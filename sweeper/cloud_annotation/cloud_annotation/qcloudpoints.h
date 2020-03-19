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
  pcl::PointXYZ beginPoint() {return begin_point_;}
  pcl::PointXYZ endPoint() {return end_point_;}
  const QList<QSharedPointer<MapPoint>> & reference() const {return reference_;}

  pcl::PointXYZ beginPointDisplay() {return begin_point_display_;}
  pcl::PointXYZ endPointDisplay() {return end_point_display_;}
  void setPointDisplay(pcl::PointXYZ, pcl::PointXYZ);

  void setOrigin(const Point &);
  void setHdMap(const HdMapRaw &);
  const HdMapRaw & hdMap() const;
  void setSelectedItem(int);
  void clear();

  void calcRoadSide(double, double, Road *);

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

  pcl::PointXYZ begin_point_display_;
  pcl::PointXYZ end_point_display_;

  Point origin_point_;
  HdMapRaw hdmap_;
};

#endif // QCLOUDPOINTS_H
