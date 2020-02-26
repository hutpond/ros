#include "qcloudpoints.h"

#include<pcl/io/io.h>
#include<pcl/io/ply_io.h>
#include<pcl/point_types.h>

QCloudPoints::QCloudPoints(QObject *parent) : QObject(parent)
{
  m_pCloudPoints.reset(new pcl::PointCloud<pcl::PointXYZ>);
}

void QCloudPoints::openFile(const QString &fileName)
{
  m_pCloudPoints->clear();
  if (pcl::io::loadPLYFile<pcl::PointXYZ>(fileName.toLocal8Bit().data(), *m_pCloudPoints) == -1) {
    return;
  }
}

pcl::PointCloud<pcl::PointXYZ>::ConstPtr QCloudPoints::points() const
{
  return m_pCloudPoints;
}
