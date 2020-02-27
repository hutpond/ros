#ifndef QCLOUDPOINTS_H
#define QCLOUDPOINTS_H

#include <QObject>
#include<pcl/io/io.h>

class QCloudPoints : public QObject
{
  Q_OBJECT
public:
  explicit QCloudPoints(QObject *parent = nullptr);

  void openFile(const QString &);
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr points() const;
  pcl::PointXYZ begin_point() {return begin_point_;}
  pcl::PointXYZ end_point() {return end_point_;}

signals:

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_points_;
  pcl::PointXYZ begin_point_;
  pcl::PointXYZ end_point_;
};

#endif // QCLOUDPOINTS_H
