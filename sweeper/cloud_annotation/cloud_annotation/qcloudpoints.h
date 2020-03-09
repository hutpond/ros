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
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points() const;
  pcl::PointXYZ begin_point() {return begin_point_;}
  pcl::PointXYZ end_point() {return end_point_;}

  void setHdMap(const HdMap &);
  const HdMap & hdMap() const;

signals:

private:
  explicit QCloudPoints(QObject *parent = nullptr);

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
  pcl::PointXYZ begin_point_;
  pcl::PointXYZ end_point_;

  HdMap hdmap_;
};

#endif // QCLOUDPOINTS_H
