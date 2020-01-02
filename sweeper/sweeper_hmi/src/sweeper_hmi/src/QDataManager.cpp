#include <QTimer>
#include <QDebug>
#include "QDataManager.h"
#include "CHostApi4HMI.h"
#include <QDateTime>

std::array<int, 2> getSunTime(long double glat, long double glong, int year, int month, int day);

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


QDataManager::QDataManager(QObject *parent)
  : QObject(parent)
{
  m_pApi4Hmi = new CHostApi4HMI;
  m_pApi4Hmi->run();

  m_pTimer = new QTimer(this);
  connect(m_pTimer, SIGNAL(timeout()), this, SLOT(onChecked()));
}

void QDataManager::startCheck()
{
  m_nStep = 0;
  m_nType = dbAds::ISelfCheck::e_type::Chassis;
  m_nResult = dbAds::ISelfCheck::e_result::Pass;
  auto pCheck = m_pApi4Hmi->m_lpApi->GetSelfCheck(m_nType);
  pCheck->Start();

  this->clearInfos();
  m_pTimer->start(100);
}

void QDataManager::startAuto()
{
  m_pApi4Hmi->m_lpApi->SweepTo();

  QDateTime current_time=QDateTime::currentDateTime();
  int year = current_time.date().year();
  int month = current_time.date().month();
  int day = current_time.date().day();

  int _minute = current_time.time().hour() * 60 + current_time.time().minute();
  auto ret = getSunTime(31.815051, 120.0127376, year, month, day);

  std::cout << "Sun Rise at:" << std::setfill('0') << std::setw(2)<< (ret[0]/60) << ":" << std::setw(2) << (ret[0]%60) << std::endl
            << "Sun Set at :" << std::setw(2) << (ret[1]/60) << ":" << std::setw(2) << (ret[1]%60) << std::endl
            << "Now at     :" << std::setw(2) << (_minute/60) << ":" << std::setw(2) << (_minute%60) << std::endl;

  if((_minute < (ret[0] + 60)) || (_minute > (ret[1] - 60)))
  {
      static const std::map<std::string, boost::any> control_ =
      {
          {dbAds::IApi4HMI::Item_spout_water, 1},
          {dbAds::IApi4HMI::Item_brush_status, 1},
          {dbAds::IApi4HMI::Item_width_light, 1},
          {dbAds::IApi4HMI::Item_low_beam_light, 1},
          {dbAds::IApi4HMI::Item_high_beam_light, 0},
          {dbAds::IApi4HMI::Item_light, 1},
          {dbAds::IApi4HMI::Item_reverse_light, 0},
          {dbAds::IApi4HMI::Item_brake_light, 0},
          {dbAds::IApi4HMI::Item_left_light, 0},
          {dbAds::IApi4HMI::Item_right_light, 0},
      };
      m_pApi4Hmi->m_lpApi->SetProperty(control_);
  }
}

void QDataManager::stopAuto()
{
  m_pApi4Hmi->m_lpApi->Stop();
}

void QDataManager::pause()
{
  m_pApi4Hmi->m_lpApi->Pause();
}

void QDataManager::resume()
{
  m_pApi4Hmi->m_lpApi->Resume();
}

void QDataManager::getInfoList()
{
  m_infos.clear();
  std::vector<dbAds::IApi4HMI::CFaultInfo> faults;
  m_pApi4Hmi->m_lpApi->GetFaultList(faults);
  for (const auto &fault : faults) {
    QMsgInfo *info = new QMsgInfo;
    info->setName(QString::fromStdString(fault.szName));
    info->setCode(QString::fromStdString(fault.szCode));
    info->setDescription(QString::fromStdString(fault.szCondition));
    this->appendInfo(info);
  }
}

bool QDataManager::autoIsReady()
{
  m_pApi4Hmi->m_lpApi->AdIsReady(0);
}

bool QDataManager::autoIsQuit()
{
  m_pApi4Hmi->m_lpApi->AdNeedQuit();
}

QVariant QDataManager::getProperty(const QString &property)
{
  boost::any value = m_pApi4Hmi->m_lpApi->GetProperty(property.toStdString());
  QVariant value_ret;
  if (value.type() == typeid(int)) {
    value_ret = boost::any_cast<int>(value);
  }
  else {
    auto str = boost::any_cast<std::string>(&value);
    if (str != nullptr) {
      value_ret = QString::fromStdString(*str);
    }
  }
  return value_ret;
}

bool QDataManager::setProperty(const QString &property, const QVariant &value)
{
  boost::any property_value;
  if (value.type() == QVariant::Int) {
    property_value = value.toInt();
  }
  else if (value.type() == QVariant::String) {
    property_value = value.toString().toStdString();
  }
  m_pApi4Hmi->m_lpApi->SetProperty(property.toStdString(), property_value);
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
  auto pCheck = m_pApi4Hmi->m_lpApi->GetSelfCheck(m_nType);
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
        std::vector<dbAds::IApi4HMI::CFaultInfo> faults;
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
