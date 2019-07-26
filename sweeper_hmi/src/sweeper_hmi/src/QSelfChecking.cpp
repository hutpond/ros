#include <QTimer>
#include <QDebug>
#include "QSelfChecking.h"

QSelfChecking::QSelfChecking(QQuickItem *parent) : QQuickItem(parent)
{
  QTimer::singleShot(5000, this, &QSelfChecking::onCheckSys);
  QTimer::singleShot(6000, this, &QSelfChecking::onCheckSensor);
  QTimer::singleShot(8000, this, &QSelfChecking::onCheckEnv);
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

