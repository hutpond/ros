#include <QTimer>
#include <QDebug>
#include "QSelfChecking.h"

QSelfChecking::QSelfChecking(QQuickItem *parent) : QQuickItem(parent)
{
  QTimer::singleShot(2000, this, &QSelfChecking::onCheckSys);
  QTimer::singleShot(3000, this, &QSelfChecking::onCheckSensor);
  QTimer::singleShot(3100, this, &QSelfChecking::onCheckEnv);
}

QSelfChecking::StepCheck QSelfChecking::step() const
{
  return m_stepCheck;
}

void QSelfChecking::setStep(StepCheck step)
{
  m_stepCheck = step;
}

void QSelfChecking::onCheckSys()
{
  this->setStep(StepSysSucceed);
  emit stepChanged();
}

void QSelfChecking::onCheckSensor()
{
  this->setStep(StepSensorFailed);
  emit stepChanged();
}

void QSelfChecking::onCheckEnv()
{
  this->setStep(StepEnvSucceed);
  emit stepChanged();
}

