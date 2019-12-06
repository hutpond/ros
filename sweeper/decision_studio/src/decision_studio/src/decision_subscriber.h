#ifndef DECISION_SUBSCRIBER_H
#define DECISION_SUBSCRIBER_H

#include <QObject>
#include <boost/filesystem.hpp>
#include <ros/ros.h>

#include <decision_studio/ads_DecisionData4Debug.h>

class QTimer;
class QCentralWidget;

class DecisionSubscriber : public QObject
{
public:
  explicit DecisionSubscriber(QCentralWidget &, QObject * = Q_NULLPTR);
  ~DecisionSubscriber();

  void setSaveFileFlag(bool);
  void openReplayDir(const QString &);
  void createSavePath(const boost::filesystem::path &);

  void setLiving(bool);
  int sizeOfReplayFiles();

protected:
  void onSubscribeData(const decision_studio::ads_DecisionData4Debug &);
  void saveDataToFile(const decision_studio::ads_DecisionData4Debug &);
  void readDataFromFile(const std::string &);

  std::string timeToString(int = 0);
  void fileList(const std::string &, std::vector<std::string> &);

  void replayIndex(int);
  void setReplayVelocity(int);

protected slots:
  void onSpin();
  void onReplay();

public slots:
  void onPlayClicked(int, int);

private:
  ros::NodeHandle m_nodeHandle;
  ros::Subscriber m_subscriber;

  QTimer *m_pTimerSubscribe;
  QTimer *m_pTimerReplay;
  QCentralWidget &m_rWdgCentral;

  bool m_bFlagLiveing;
  bool m_bFlagSaveFile;
  boost::filesystem::path m_fsPath;

  int m_nReplayIntervalIndex;
  bool m_bFlagReplayPause;
  std::vector<std::string> m_listPlanningFiles;
  std::vector<std::string>::iterator m_itFile;  // 文件名链表迭代器
};

#endif // DECISION_SUBSCRIBER_H
