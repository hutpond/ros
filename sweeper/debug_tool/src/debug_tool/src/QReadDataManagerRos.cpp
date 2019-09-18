#include "QReadDataManagerRos.h"
#include "GlobalDefine.h"

QReadDataManagerRos QReadDataManagerRos::s_instance;

QReadDataManagerRos * QReadDataManagerRos::instance()
{
  return &s_instance;
}

QReadDataManagerRos::QReadDataManagerRos(QObject *parent)
  : QObject(parent)
{
}

/**
 * @brief 开始订阅ros消息
 * @param

 * @return
 */
void QReadDataManagerRos::start_subscribe()
{
  m_pNodeHandle.reset(new ros::NodeHandle);
  m_subPlanning = m_pNodeHandle->subscribe(
        "planning_debug_data", 10,
        &QReadDataManagerRos::on_planning_subscirbe, this);

  m_subPlanningNew = m_pNodeHandle->subscribe(
        "planning_debug_data_new", 10,
        &QReadDataManagerRos::on_planning_subscirbe_new, this);

  m_nTimerId = startTimer(100);
}

/**
 * @brief 停止订阅ros消息
 * @param

 * @return
 */
void QReadDataManagerRos::stop_subscirbe()
{
  killTimer(m_nTimerId);
  m_nTimerId = 0;
  m_pNodeHandle->shutdown();
}

void QReadDataManagerRos::timerEvent(QTimerEvent *)
{
  ros::spinOnce();
}

/**
 * @brief planning数据的ros订阅回调函数
 * @param msg: ros msg

 * @return
 */
void QReadDataManagerRos::on_planning_subscirbe(const debug_tool::ads_PlanningData4Debug &data)
{
  emit planningData(data);
}

void QReadDataManagerRos::on_planning_subscirbe_new(const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  emit planningData(data);
}
