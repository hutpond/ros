#ifndef QREADDATAROSOBJECT_H
#define QREADDATAROSOBJECT_H

#include <memory>
#include <QObject>
#include <ros/ros.h>
#include "path_editor/ads_ins_data.h"

class QReadDataRosObject : public QObject
{
  Q_OBJECT
public:
  explicit QReadDataRosObject(QObject *parent = nullptr);

  void startSubscribe();
  void stopSubscribe();

signals:
  void imuData(const path_editor::ads_ins_data::ConstPtr &);

public slots:

protected:
  void timerEvent(QTimerEvent *);

  void on_subscribe_imu(const path_editor::ads_ins_data::ConstPtr &);

private:
  std::unique_ptr<ros::NodeHandle> m_nodeHandle;
  ros::Subscriber m_subscirber;

  int m_nTimerId;
};

#endif // QREADDATAROSOBJECT_H
