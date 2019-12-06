#include <fstream>
#include <QTimer>

#include "decision_subscriber.h"
#include "qcentralwidget.h"
#include "qreplaywidget.h"

constexpr int REPLAY_INTERVAL[] = {10, 50, 250, 500};

DecisionSubscriber::DecisionSubscriber(QCentralWidget &widget, QObject *parent)
  : m_rWdgCentral(widget)
  , QObject(parent)
  , m_bFlagLiveing(true)
  , m_bFlagSaveFile(false)
  , m_nReplayIntervalIndex(sizeof(REPLAY_INTERVAL) - 1)
  , m_bFlagReplayPause(false)
{
  m_subscriber = m_nodeHandle.subscribe(
        "topic_point_cloud", 10,
        &DecisionSubscriber::onSubscribeData, this);

  m_pTimerSubscribe = new QTimer(this);
  connect(m_pTimerSubscribe, &QTimer::timeout, this, &DecisionSubscriber::onSpin);

  m_pTimerReplay = new QTimer(this);
  connect(m_pTimerReplay, &QTimer::timeout, this, &DecisionSubscriber::onReplay);

  this->setLiving(true);
}

DecisionSubscriber::~DecisionSubscriber()
{
  m_pTimerSubscribe->stop();
  m_pTimerReplay->stop();
}

void DecisionSubscriber::setSaveFileFlag(bool flag)
{
  m_bFlagSaveFile = flag;
}

void DecisionSubscriber::openReplayDir(const QString &path)
{
  m_bFlagReplayPause = false;
  this->fileList(path.toStdString(), m_listPlanningFiles);
  m_itFile = m_listPlanningFiles.begin();

  this->setLiving(false);
}

void DecisionSubscriber::onSpin()
{
  ros::spinOnce();
}

void DecisionSubscriber::onReplay()
{
  if (!m_bFlagLiveing && !m_bFlagReplayPause && m_itFile != m_listPlanningFiles.end()) {
    this->readDataFromFile(*m_itFile);
    ++ m_itFile;
  }
}

void DecisionSubscriber::onSubscribeData(const decision_studio::ads_DecisionData4Debug &data)
{
  if (m_bFlagLiveing) {
    m_rWdgCentral.setData(data);
  }
  if (m_bFlagSaveFile) {
    this->saveDataToFile(data);
  }
}

void DecisionSubscriber::saveDataToFile(const decision_studio::ads_DecisionData4Debug &data)
{
  std::string fileName = m_fsPath.string();
  if (fileName[fileName.size() - 1] != '/') {
    fileName.push_back('/');
  }
  fileName += this->timeToString();
  fileName += ".txt";
  std::ofstream out(fileName);
  out << data;
  out.close();
}

void DecisionSubscriber::readDataFromFile(const std::string &name)
{
  decision_studio::ads_DecisionData4Debug data;
  std::ifstream in(name);
  //in >> data;
  in.close();
  m_rWdgCentral.setData(data);
}

void DecisionSubscriber::createSavePath(const boost::filesystem::path &path)
{
  m_fsPath = path;
  m_fsPath /= this->timeToString(1);
  boost::filesystem::create_directories(m_fsPath);
}

void DecisionSubscriber::setLiving(bool flag)
{
  m_bFlagLiveing = flag;
  if (flag) {
    m_pTimerSubscribe->start(20);
    m_pTimerReplay->stop();
  }
  else {
    m_pTimerSubscribe->stop();
    m_pTimerReplay->start(REPLAY_INTERVAL[m_nReplayIntervalIndex]);
  }
}

void DecisionSubscriber::onPlayClicked(int index, int value)
{
  switch (index) {
  case QReplayWidget::BtnPlayPause:
    m_bFlagReplayPause = (value == 0);
    break;
  case QReplayWidget::BtnPrevious:
  case QReplayWidget::BtnNext:
    this->replayIndex(value);
    break;
  case QReplayWidget::BtnPlayVelocity:
    this->setReplayVelocity(value);
    break;
  case QReplayWidget::BtnCount:
    this->replayIndex(value);
    break;
  default:
    break;
  }
}

void DecisionSubscriber::replayIndex(int index)
{
  if (!m_bFlagLiveing && m_bFlagReplayPause && index >= 0 &&
      index < m_listPlanningFiles.size()) {
    m_itFile = m_listPlanningFiles.begin();
    std::advance(m_itFile, index);
    this->readDataFromFile(*m_itFile);
  }
}

void DecisionSubscriber::setReplayVelocity(int value)
{
  if (value < 0 || value >= sizeof(REPLAY_INTERVAL) - 1 && m_nReplayIntervalIndex == value) {
    return;
  }
  m_nReplayIntervalIndex = value;
  m_pTimerReplay->setInterval(REPLAY_INTERVAL[m_nReplayIntervalIndex]);
}

int DecisionSubscriber::sizeOfReplayFiles()
{
  return m_listPlanningFiles.size();
}

std::string DecisionSubscriber::timeToString(int type)
{
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);

  struct timeval tv;
  gettimeofday(&tv, NULL);

  char time_sz[64] = {0};
  std::string time_str;
  if (type == 0) {
    sprintf(time_sz, "%04d%02d%02d_%02d%02d%02d_%3d",
            utcTime->tm_year + 1900,
            utcTime->tm_mon + 1,
            utcTime->tm_mday,
            utcTime->tm_hour,
            utcTime->tm_min,
            utcTime->tm_sec,
            static_cast<int>((tv.tv_usec / 1000) % 1000)
            );
  }
  else {
    sprintf(time_sz, "%04d%02d%02d_%02d%02d%02d",
            utcTime->tm_year + 1900,
            utcTime->tm_mon + 1,
            utcTime->tm_mday,
            utcTime->tm_hour,
            utcTime->tm_min,
            utcTime->tm_sec
            );
  }
  time_str = time_sz;

  return time_str;
}

/**
 * @brief 获取某路径下所有数据文件名
 * @param path: 路径名

 * @return: 数据文件名字符串链表
 */
void DecisionSubscriber::fileList(const std::string &path, std::vector<std::string> &files)
{
  files.clear();
  namespace fs = boost::filesystem;
  fs::directory_iterator end_iter;
  for (fs::directory_iterator it(path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
    std::string name = ss.str();
    std::string substr = name.substr(name.size() - 4, 3);
    if (substr != "txt") {
      continue;
    }
    files.push_back(ss.str());
  }
  std::sort(files.begin(), files.end());
}

