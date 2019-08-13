#include <QTimer>
#include <QDebug>
#include "QDataManager.h"
#include "CHostApi4HMI.h"

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
  m_pApi4Hmi = new CHostApi4HMI;
  m_pTimer = new QTimer(this);
  connect(m_pTimer, SIGNAL(timeout()), this, SLOT(onChecked()));
}

void QDataManager::startCheck()
{
  m_nStep = 0;
  m_nType = dbAds::ISelfCheck::e_type::Vehicle;
  m_nResult = dbAds::ISelfCheck::e_result::Pass;
  dbAds::ISelfCheck *pCheck = m_pApi4Hmi->m_lpApi->GetSelfCheck(m_nType);
  pCheck->Start();

  this->clearInfos();
  m_pTimer->start(100);
}

void QDataManager::startAuto()
{
  m_pApi4Hmi->m_lpApi->StartAutoDrive();
}

void QDataManager::stopAuto()
{
  m_pApi4Hmi->m_lpApi->StopAutoDrive();
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
  dbAds::ISelfCheck *pCheck = m_pApi4Hmi->m_lpApi->GetSelfCheck(m_nType);
  int percent = pCheck->GetProgress();
  m_nStep = 25 * m_nType + percent / 4;
  emit stepChanged(m_nStep);

  dbAds::ISelfCheck::e_result nResult = pCheck->GetResult();
  if (m_nResult == dbAds::ISelfCheck::e_result::Pass &&
      nResult == dbAds::ISelfCheck::e_result::Failure) {
    m_nResult = nResult;
  }
  if (nResult == dbAds::ISelfCheck::e_result::Pass ||
      nResult == dbAds::ISelfCheck::e_result::Failure) {

    emit checkEnd((CheckType)(m_nType), nResult == dbAds::ISelfCheck::e_result::Pass);
    pCheck->Stop();

    if (m_nType == dbAds::ISelfCheck::e_type::Algorithm) {
      m_pTimer->stop();
      QTimer::singleShot(1000, this, SIGNAL(stopCheck()));

      if (m_nResult != 0) {
        std::vector<dbAds::IApi4HMI::CFault> faults;
        m_pApi4Hmi->m_lpApi->GetFaultList(faults);
        for (const auto &fault : faults) {
          QMsgInfo *info = new QMsgInfo;
          info->setName(QString::fromStdString(fault.szName));
          info->setCode(QString::fromStdString(fault.szCode));
          info->setDescription(QString::fromStdString(fault.szCondition));
          this->appendInfo(info);
        }
      }
    }
    else {
      m_nType = (dbAds::ISelfCheck::e_type) (int(m_nType) + 1);
      pCheck = m_pApi4Hmi->m_lpApi->GetSelfCheck(m_nType);
      pCheck->Start();
    }
  }
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
