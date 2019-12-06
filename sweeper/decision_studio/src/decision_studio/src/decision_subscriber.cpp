#include <QTimer>

#include "decision_subscriber.h"
#include "qcentralwidget.h"

DecisionSubscriber::DecisionSubscriber(QCentralWidget &widget, QObject *parent)
  : m_rWdgCentral(widget)
  , QObject(parent)
{
  m_subscriber = m_nodeHandle.subscribe(
        "topic_point_cloud", 10,
        &DecisionSubscriber::onSubscribeData, this);

  m_pTimer = new QTimer(this);
  connect(m_pTimer, &QTimer::timeout, this, &DecisionSubscriber::onSpin);
  m_pTimer->start(20);
}

DecisionSubscriber::~DecisionSubscriber()
{
  m_pTimer->stop();
}

void DecisionSubscriber::onSpin()
{
  ros::spinOnce();
}

void DecisionSubscriber::onSubscribeData(const decision_studio::ads_DecisionData4Debug &data)
{
  m_rWdgCentral.setData(data);
}
