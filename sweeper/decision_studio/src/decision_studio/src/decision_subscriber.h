#ifndef DECISION_SUBSCRIBER_H
#define DECISION_SUBSCRIBER_H

#include <QObject>
#include <ros/ros.h>

#include <decision_studio/ads_DecisionData4Debug.h>

class QTimer;
class QCentralWidget;

class DecisionSubscriber : public QObject
{
public:
  explicit DecisionSubscriber(QCentralWidget &, QObject * = Q_NULLPTR);
  ~DecisionSubscriber();

protected:
  void onSubscribeData(const decision_studio::ads_DecisionData4Debug &);

protected slots:
  void onSpin();

private:
  ros::NodeHandle m_nodeHandle;
  ros::Subscriber m_subscriber;

  QTimer *m_pTimer;
  QCentralWidget &m_rWdgCentral;
};

#endif // DECISION_SUBSCRIBER_H
