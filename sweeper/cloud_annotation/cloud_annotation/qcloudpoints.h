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

signals:

private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_pCloudPoints;
};

#endif // QCLOUDPOINTS_H
