#include "QReadDataManagerRos.h"
#include "GlobalDefine.h"
#include "QPlanningWidget.h"
#include "QNewPlanningWidget.h"

QReadDataManagerRos QReadDataManagerRos::s_instance;

QReadDataManagerRos * QReadDataManagerRos::instance()
{
  return &s_instance;
}

QReadDataManagerRos::QReadDataManagerRos(QObject *parent)
  : QObject(parent)
  , m_nTimerId(0)
  , m_pWdgNewPlanning(Q_NULLPTR)
  , m_pWdgPlanning(Q_NULLPTR)
{
}

/**
 * @brief 开始订阅ros消息
 * @param

 * @return
 */
void QReadDataManagerRos::start_subscribe()
{
  if (m_nTimerId != 0) {
    return;
  }
  m_pNodeHandle.reset(new ros::NodeHandle);
  m_subPlanning = m_pNodeHandle->subscribe(
        "planning_debug_data", 10,
        &QReadDataManagerRos::on_planning_subscirbe, this);

  m_subPlanningNew = m_pNodeHandle->subscribe(
        "planning_debug_data_new", 10,
        &QReadDataManagerRos::on_planning_subscirbe_new, this);

  m_nTimerId = startTimer(10);
}

/**
 * @brief 停止订阅ros消息
 * @param

 * @return
 */
void QReadDataManagerRos::stop_subscirbe()
{
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
    m_pNodeHandle->shutdown();
  }
}

void QReadDataManagerRos::setPlanningWidget(QPlanningWidget *widget)
{
  m_pWdgPlanning = widget;
}

void QReadDataManagerRos::setNewPlanningWidget(QNewPlanningWidget *widget)
{
  m_pWdgNewPlanning = widget;
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
  if (m_pWdgPlanning != Q_NULLPTR) {
    m_pWdgPlanning->onParsePlanningData(data);
  }
}

void QReadDataManagerRos::on_planning_subscirbe_new(const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  if (m_pWdgNewPlanning != Q_NULLPTR) {
    m_pWdgNewPlanning->onParsePlanningData(data);
  }
}
