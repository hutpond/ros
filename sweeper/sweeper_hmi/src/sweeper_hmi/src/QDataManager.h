#ifndef QDATAMANAGER_H
#define QDATAMANAGER_H

#include <QObject>
#include <QVector>
#include <QPointF>
#include <QQmlListProperty>
#include "dbAdsApi4HMI.h"

class QTimer;
class CHostApi4HMI;

class QMsgInfo : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString name READ name WRITE setName)
  Q_PROPERTY(QString code READ code WRITE setCode)
  Q_PROPERTY(QString description READ description WRITE setDescription)

public:
  QMsgInfo(QObject *parent = Q_NULLPTR);
  ~QMsgInfo();

  QString name() const;
  void setName(const QString &);
  QString code() const;
  void setCode(const QString &);
  QString description() const;
  void setDescription(const QString &);

private:
  QString m_strName;
  QString m_strCode;
  QString m_strDescription;
};


class QDataManager : public QObject
{
  Q_OBJECT
  Q_PROPERTY(int step READ step WRITE setStep NOTIFY stepChanged)
  Q_PROPERTY(QQmlListProperty<QMsgInfo> infos READ infos)
  Q_ENUMS(CheckType)

public:
  enum CheckType
  {
    CheckVehicle,
    CheckSystem,
    CheckSensor,
    CheckAlgorithm
  };

public:
  explicit QDataManager(QObject *parent = nullptr);

  Q_INVOKABLE void startCheck();
  Q_INVOKABLE void startAuto();
  Q_INVOKABLE void stopAuto();
  Q_INVOKABLE void pause();
  Q_INVOKABLE void resume();
  Q_INVOKABLE void stopBySide();

  Q_INVOKABLE void getInfoList();

  Q_INVOKABLE bool autoIsReady();
  Q_INVOKABLE bool autoIsQuit();

  Q_INVOKABLE QVariant getProperty(const QString &);
  Q_INVOKABLE bool setProperty(const QString &, const QVariant &);

  Q_INVOKABLE QPointF llaToEnu(const QPointF &, const QPointF &);
  Q_INVOKABLE QVariantList getTargets();
  Q_INVOKABLE QVariantList getCleanningRoute();
  Q_INVOKABLE bool setCleaningTask(const QString &, const QString &);

  int step() const;
  void setStep(int);

  QQmlListProperty<QMsgInfo> infos();
  void appendInfo(QMsgInfo *info);
  int infoCount() const;
  QMsgInfo * infoAt(int index) const;
  void clearInfos();

signals:
  void checkEnd(CheckType type, bool ret);
  void stepChanged(int step);
  void stopCheck();

public slots:
  void onChecked();

private:
  int m_nStep;
  dbAds::ISelfCheck::e_type m_nType;
  dbAds::ISelfCheck::e_result m_nResult;
  QVector<QMsgInfo *> m_infos;

  QTimer *m_pTimer;
  CHostApi4HMI *m_pApi4Hmi;

  static void appendInfo(QQmlListProperty<QMsgInfo> *list, QMsgInfo *info);
  static void clearInfos(QQmlListProperty<QMsgInfo> *list);
  static QMsgInfo * infoAt(QQmlListProperty<QMsgInfo> *list, int index);
  static int infoCount(QQmlListProperty<QMsgInfo> *list);
};


#endif // QDATAMANAGER_H
