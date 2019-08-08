#ifndef Q_READ_DATA_MANAGER_ROS_H
#define Q_READ_DATA_MANAGER_ROS_H

#include <boost/smart_ptr/make_shared.hpp>
#include <QObject>
#include "ros/ros.h"
#include "debug_tool/PlanningData4Debug.h"
#include "debug_tool/TargetPoint.h"
#include "debug_tool/ReferencePoint.h"

class QReadDataManagerRos : public QObject
{
  Q_OBJECT
public:
  static QReadDataManagerRos * instance();
  void start_subscribe();
  void stop_subscirbe();

  void send_planning_data(const debug_tool::PlanningData4Debug &);

protected:
  void timerEvent(QTimerEvent *);

private:
  explicit QReadDataManagerRos(QObject *parent = NULL);

signals:
  void planningData(const debug_tool::PlanningData4Debug &);

public slots:

protected:
  int m_nTimerId;
  boost::shared_ptr<ros::NodeHandle> m_pNodeHandle;
  ros::Subscriber m_subPlanning;

  static QReadDataManagerRos s_instance;
};

#endif // Q_READ_DATA_MANAGER_ROS_H
