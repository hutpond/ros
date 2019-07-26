#ifndef QSELFCHECKING_H
#define QSELFCHECKING_H

#include <QQuickItem>

class QSelfChecking : public QQuickItem
{
  Q_OBJECT
  Q_PROPERTY(StepCheck step READ step WRITE setStep NOTIFY stepChanged)
  Q_ENUMS(StepCheck)

public:
  enum StepCheck
  {
    StepSysChecking,
    StepSysSucceed,
    StepSysFailed,
    StepSensorChecking,
    StepSensorSucceed,
    StepSensorFailed,
    StepEnvChecking,
    StepEnvSucceed,
    StepEnvFailed,
  };

public:
  explicit QSelfChecking(QQuickItem *parent = nullptr);
  StepCheck step() const;
  void setStep(StepCheck);

signals:
  void stepChanged();

private slots:
  void onCheckSys();
  void onCheckSensor();
  void onCheckEnv();

private:
  StepCheck m_stepCheck;
};

#endif // QSELFCHECKING_H
