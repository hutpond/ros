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
struct Point;

class QOpenDriveObject : public QObject
{
  Q_OBJECT
public:
  explicit QOpenDriveObject(QObject *parent = nullptr);

  void readOpenDriveFile(const QString &, const QString &, QList<QSharedPointer<Point>> &);
  void writeOpenDriveFile(const QString &, const QString &, const QList<QSharedPointer<Point>> &);

signals:

public slots:

private:
};

#endif // QOPENDRIVEOBJECT_H
