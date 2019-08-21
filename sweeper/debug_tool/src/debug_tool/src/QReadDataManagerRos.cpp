#include "QReadDataManagerRos.h"
#include "GlobalDefine.h"

void on_planning_subscirbe(const debug_tool::ads_PlanningData4Debug &);

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
        on_planning_subscirbe);

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
 * @brief 发送planning数据
 * @param data: json字符串

 * @return
 */
void QReadDataManagerRos::send_planning_data(const debug_tool::ads_PlanningData4Debug &data)
{
  emit planningData(data);
}

/**
 * @brief planning数据的ros订阅回调函数
 * @param msg: ros msg

 * @return
 */
void on_planning_subscirbe(const debug_tool::ads_PlanningData4Debug &data)
{
  QReadDataManagerRos::instance()->send_planning_data(data);
}

