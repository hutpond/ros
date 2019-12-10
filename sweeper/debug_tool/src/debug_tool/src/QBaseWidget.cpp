/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 子模块基类
********************************************************/
#include "QBaseWidget.h"

#include <sstream>
#include "QPlanningParamWidget.h"
#include "QFullViewWidget.h"
#include "QReadDataManagerRos.h"
#include "QNewPlanningPlot.h"

static constexpr int REPLAY_MSEC[3] = {50, 200, 400};

QBaseWidget::QBaseWidget(QWidget *parent)
  : QWidget(parent)
  , m_pWdgPlotting(Q_NULLPTR)
  , m_nTimerId(0)
{
}

QBaseWidget::~QBaseWidget()
{
}

void QBaseWidget::resizeEvent(QResizeEvent *)
{
  const int W_PERCENT = 70;
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const int WDG_SHWO_W = WIDTH * W_PERCENT / 200;
  if (m_pWdgShow[1] != Q_NULLPTR) {
    for (int i = 0; i < 2; ++i) {
      m_pWdgShow[i]->setGeometry(
            i * WDG_SHWO_W,
            0,
            WDG_SHWO_W,
            HEIGHT
            );
    }
  }
  else {
    m_pWdgShow[0]->setGeometry(
          0,
          0,
          WDG_SHWO_W * 2,
          HEIGHT
          );
  }
  if (m_pWdgPlotting != Q_NULLPTR) {
    m_pWdgPlotting->setGeometry(
          0,
          0,
          WDG_SHWO_W * 2,
          HEIGHT
          );
  }
  m_pWdgFullView->setGeometry(
        0,
        0,
        WIDTH * W_PERCENT / 100,
        HEIGHT
        );
  m_pWdgParam->setGeometry(
        WIDTH * W_PERCENT / 100,
        0,
        WIDTH * (100 - W_PERCENT) / 100,
        HEIGHT
        );
}

/*******************************************************
 * @brief 获取某路径下所有数据文件名
 * @param path: 路径名

 * @return: 数据文件名字符串链表
********************************************************/
void QBaseWidget::fileList(const std::string &path, std::vector<std::string> &files)
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

/*******************************************************
 * @brief replay暂停状态
 * @param pause: 暂停状态, true: 停, false: 恢复

 * @return
********************************************************/
void QBaseWidget::onReplayState(bool pause)
{
  m_bFlagPauseReplay = pause;
}


int QBaseWidget::showType()
{
  return m_nShowType;
}

void QBaseWidget::setShowType(int type)
{
  m_nShowType = type;
  m_pWdgParam->setShowType(type);

  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
  if (type == RePlay) {
    m_nTimerId = startTimer(REPLAY_MSEC[m_nReplaySpeedIndex]);
  }
}

int QBaseWidget::showView()
{
  return m_nShowView;
}

void QBaseWidget::setReplaySpeedIndex(int index)
{
  m_nReplaySpeedIndex = index;
  if (m_nTimerId == 0) {
    return;
  }
  killTimer(m_nTimerId);
  m_nTimerId = startTimer(REPLAY_MSEC[index]);
}

int QBaseWidget::replaySpeedIndex()
{
  return m_nReplaySpeedIndex;
}

/*******************************************************
 * @brief 缩放图像显示
 * @param index: -1， 缩小, 0, 复原, 1, 放大

 * @return
********************************************************/
void QBaseWidget::setViewResolution(int index)
{
  switch (m_nShowView) {
    case LocalViewVehicle:
    case LocalViewENU:
    case LocalViewFrenet:
      m_pWdgShow[0]->setViewResolution(index);
      if (m_pWdgShow[1] != Q_NULLPTR) {
        m_pWdgShow[1]->setViewResolution(index);
      }
      break;
    case FullView:
      m_pWdgFullView->setViewResolution(index);
      break;
    default:
      break;
  }
}

/*******************************************************
 * @brief 开始重放
 * @param index: 重放序号
 * @param path: 重放文件路径

 * @return
********************************************************/
void QBaseWidget::startReplay(const QString &path)
{
  m_bFlagPauseReplay = true;
  m_listPlanningFiles.clear();
  this->fileList(path.toStdString(), m_listPlanningFiles);
  m_itFile = m_listPlanningFiles.begin();
  m_pWdgParam->setFrameCount(m_listPlanningFiles.size());

  m_pWdgFullView->clearMapDatas();
  boost::filesystem::path fs_path = path.toStdString();
  m_pWdgFullView->loadReferenceFile(fs_path);

  m_bFlagPauseReplay = false;
}

/*******************************************************
 * @brief 停止显示线程
 * @param

 * @return
********************************************************/
void QBaseWidget::stopDisplay()
{
  QReadDataManagerRos::instance()->stop_subscirbe();
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
}

/*******************************************************
 * @brief 是否显示所有target
 * @param show: true, 显示所有

 * @return
********************************************************/
void QBaseWidget::setShowAllTargets(bool show)
{
  m_pWdgShow[0]->setShowAllTargets(show);
  if (m_pWdgShow[1] != Q_NULLPTR) {
    m_pWdgShow[1]->setShowAllTargets(show);
  }
}

void QBaseWidget::onSelectTool(int index, bool checkable)
{
  m_pWdgShow[LivePlay]->setToolIndex(index, checkable);
  if (m_pWdgShow[1] != Q_NULLPTR) {
    m_pWdgShow[RePlay]->setToolIndex(index, checkable);
  }
}

std::string QBaseWidget::dataFileName()
{
  char time_str[64] = {0};
  time_t times = time(NULL);
  struct tm *utcTime = localtime(&times);
  struct timeval tv;
  gettimeofday(&tv, NULL);
  sprintf(time_str, "%04d%02d%02d_%02d%02d%02d_%03d",
          utcTime->tm_year + 1900,
          utcTime->tm_mon + 1,
          utcTime->tm_mday,
          utcTime->tm_hour,
          utcTime->tm_min,
          utcTime->tm_sec,
          static_cast<int>((tv.tv_usec / 1000) % 1000)
          );

  std::string strFileName = time_str;
  strFileName += ".txt";
  return strFileName;
}
