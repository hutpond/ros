#include <QTimer>
#include <QRandomGenerator>
#include <QDebug>
#include "QDataManager.h"

QMsgInfo::QMsgInfo(QObject *parent)
  : QObject(parent) {

}

QMsgInfo::~QMsgInfo()
{

}

QString QMsgInfo::name() const
{
  return m_strName;
}

void QMsgInfo::setName(const QString &name)
{
  m_strName = name;
}

QString QMsgInfo::code() const
{
  return m_strCode;
}

void QMsgInfo::setCode(const QString &code)
{
  m_strCode = code;
}

QString QMsgInfo::description() const
{
  return m_strDescription;
}

void QMsgInfo::setDescription(const QString &description)
{
  m_strDescription = description;
}


QVector<QMsgInfo *> QDataManager::m_infos;
QDataManager::QDataManager(QObject *parent)
  : QObject(parent)
{
  m_pTimer = new QTimer(this);
  connect(m_pTimer, SIGNAL(timeout()), this, SLOT(onChecked()));
}

void QDataManager::startCheck()
{
  m_nStep = 0;
  this->clearInfos();
  m_pTimer->start(100);
}

void QDataManager::startAuto()
{

}

void QDataManager::stopAuto()
{

}

int QDataManager::step() const
{
  return m_nStep;
}

void QDataManager::setStep(int step)
{
  m_nStep = step;
}

QQmlListProperty<QMsgInfo> QDataManager::infos()
{
  return QQmlListProperty<QMsgInfo>(
        this, this,
        &QDataManager::appendInfo,
        &QDataManager::infoCount,
        &QDataManager::infoAt,
        &QDataManager::clearInfos);
}

void QDataManager::appendInfo(QMsgInfo *info)
{
  m_infos.append(info);
}

int QDataManager::infoCount() const
{
  return m_infos.size();
}

QMsgInfo * QDataManager::infoAt(int index) const
{
  return m_infos.at(index);
}

void QDataManager::clearInfos()
{
  m_infos.clear();
}

void QDataManager::onChecked()
{
  if (m_nStep == 25) {
    int w = QRandomGenerator::global()->bounded(10);
    emit checkEnd(CheckVehicle, (w != 0));
  }
  else if (m_nStep == 60) {
    int w = QRandomGenerator::global()->bounded(10);
    emit checkEnd(CheckSystem, (w != 0));
  }
  else if (m_nStep == 70) {
    int w = QRandomGenerator::global()->bounded(10);
    emit checkEnd(CheckSensor, (w != 0));
  }
  else if (m_nStep == 100) {
    int w = QRandomGenerator::global()->bounded(10);
    emit checkEnd(CheckAlgorithm, (w != 0));
    m_pTimer->stop();
    QTimer::singleShot(1000, this, SIGNAL(stopCheck()));

    for (int i = 0; i < 12; ++i) {
      QMsgInfo *info = new QMsgInfo;
      info->setName(QString("Name %1").arg(i + 1));
      info->setCode(QString::number(1000 + i));
      info->setDescription("Description Test show something.");
      this->appendInfo(info);
    }
  }
  emit stepChanged(m_nStep);
  ++ m_nStep;
}

void QDataManager::appendInfo(QQmlListProperty<QMsgInfo> *list, QMsgInfo *info)
{
  reinterpret_cast<QDataManager *>(list->data)->appendInfo(info);
}

void QDataManager::clearInfos(QQmlListProperty<QMsgInfo> *list)
{
  reinterpret_cast<QDataManager *>(list->data)->clearInfos();
}

QMsgInfo * QDataManager::infoAt(QQmlListProperty<QMsgInfo> *list, int index)
{
  return reinterpret_cast<QDataManager *>(list->data)->infoAt(index);
}

int QDataManager::infoCount(QQmlListProperty<QMsgInfo> *list)
{
  return reinterpret_cast<QDataManager *>(list->data)->infoCount();
}
