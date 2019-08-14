#include "QReadDataRosObject.h"

QReadDataRosObject::QReadDataRosObject(QObject *parent)
  : QObject(parent)
  , m_nTimerId(0)
{
  m_nodeHandle.reset(new ros::NodeHandle);
  auto fun = std::bind(&QReadDataRosObject::on_subscribe_imu, this, std::placeholders::_1);
  m_subscirber = m_nodeHandle->subscribe<path_editor::ads_ins_data>(
        "/ads_imu_data",
        5,
        fun);
}

void QReadDataRosObject::startSubscribe()
{
  m_nTimerId = startTimer(10);
}

void QReadDataRosObject::stopSubscribe()
{
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
    m_nodeHandle->shutdown();
  }
}

void QReadDataRosObject::timerEvent(QTimerEvent *)
{
 ros::spinOnce();
}

void QReadDataRosObject::on_subscribe_imu(const path_editor::ads_ins_data::ConstPtr &data)
{
  emit imuData(data);
}
