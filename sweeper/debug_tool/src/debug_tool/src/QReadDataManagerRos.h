#ifndef Q_READ_DATA_MANAGER_ROS_H
#define Q_READ_DATA_MANAGER_ROS_H

#include <boost/smart_ptr/make_shared.hpp>
#include <QObject>
#include "ros/ros.h"
#include "debug_tool/ads_PlanningData4Debug.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QReadDataManagerRos : public QObject
{
  Q_OBJECT
public:
  static QReadDataManagerRos * instance();
  void start_subscribe();
  void stop_subscirbe();

protected:
  void timerEvent(QTimerEvent *);

  void on_planning_subscirbe(const debug_tool::ads_PlanningData4Debug &);
  void on_planning_subscirbe_new(const debug_ads_msgs::ads_msgs_planning_debug_frame &);

private:
  explicit QReadDataManagerRos(QObject *parent = NULL);

signals:
  void planningData(const debug_tool::ads_PlanningData4Debug &);
  void planningDataNew(const debug_ads_msgs::ads_msgs_planning_debug_frame &);

public slots:

protected:
  int m_nTimerId;
  boost::shared_ptr<ros::NodeHandle> m_pNodeHandle;
  ros::Subscriber m_subPlanning;

  ros::Subscriber m_subPlanningNew;

  static QReadDataManagerRos s_instance;
};

#endif // Q_READ_DATA_MANAGER_ROS_H
