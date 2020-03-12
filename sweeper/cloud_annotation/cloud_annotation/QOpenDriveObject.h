/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QOpenDriveObject.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: OpenDrive解析、保存
********************************************************/
#ifndef QOPENDRIVEOBJECT_H
#define QOPENDRIVEOBJECT_H

#include <QObject>

struct InfoPacket;
struct MapPoint;
struct HdMap;

class QOpenDriveObject : public QObject
{
  Q_OBJECT
public:
  explicit QOpenDriveObject(QObject *parent = nullptr);

  void readOpenDriveFile(const QString &,
                         QList<QSharedPointer<MapPoint>> &,
                         QList<QSharedPointer<MapPoint>> &);
  void writeOpenDriveFile(const QString &,
                          const QList<QSharedPointer<MapPoint>> &,
                          const QList<QSharedPointer<MapPoint>> &);
};

#endif // QOPENDRIVEOBJECT_H
