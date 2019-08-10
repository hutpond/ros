#ifndef QDATAMANAGER_H
#define QDATAMANAGER_H

#include <QObject>
#include <QVector>
#include <QQmlListProperty>

class QTimer;

class QMsgInfo : public QObject
{
  Q_OBJECT
  Q_PROPERTY(QString name READ name WRITE setName)
  Q_PROPERTY(QString code READ code WRITE setCode)
  Q_PROPERTY(QString description READ description WRITE setDescription)

public:
  QMsgInfo(QObject *parent = 0);
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
  static QVector<QMsgInfo *> m_infos;

  QTimer *m_pTimer;

  static void appendInfo(QQmlListProperty<QMsgInfo> *list, QMsgInfo *info);
  static void clearInfos(QQmlListProperty<QMsgInfo> *list);
  static QMsgInfo * infoAt(QQmlListProperty<QMsgInfo> *list, int index);
  static int infoCount(QQmlListProperty<QMsgInfo> *list);
};


#endif // QDATAMANAGER_H