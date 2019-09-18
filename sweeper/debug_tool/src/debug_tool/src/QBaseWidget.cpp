/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 子模块基类
********************************************************/
#include <sstream>
#include "QBaseWidget.h"
#include "QPlanningParamWidget.h"
#include "QFullViewWidget.h"
#include "QReadDataManagerRos.h"

static constexpr int REPLAY_MSEC[3] = {50, 200, 400};

QBaseWidget::QBaseWidget(QWidget *parent)
  : QWidget(parent)
  , m_nTimerId(0)
{
}

QBaseWidget::~QBaseWidget()
{
}


/*******************************************************
 * @brief 重放数据文件
 * @param subdir: 子文件夹名称

 * @return
********************************************************/
void QBaseWidget::replay()
{
  std::string path = m_fsPath.string();

  m_bFlagPauseReplay = false;
  m_listPlanningFiles.clear();
  this->fileList(path, m_listPlanningFiles);

  m_itFile = m_listPlanningFiles.begin();
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
  m_bFlagPauseReplay = false;
  //m_nTimerId = startTimer(m_nIntervalMillSecs);
}

/*******************************************************
 * @brief 获取所有保存数据文件的路径
 * @param subdir: 子文件夹名称

 * @return: 路径名字符串链表
********************************************************/
std::list<std::string> QBaseWidget::pathList(const std::string &subdir)
{
  namespace fs = boost::filesystem;
  fs::path path = fs::current_path();
  path /= subdir;
  std::list<std::string> paths;
  fs::directory_iterator end_iter;
  for (fs::directory_iterator it(path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
    paths.push_back(ss.str());
  }
  return paths;
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
  fs::path fs_path = fs::path(path);
  fs::directory_iterator end_iter;
  for (fs::directory_iterator it(path); it != end_iter; ++it) {
    std::stringstream ss;
    ss << *it;
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

void QBaseWidget::changeShowView()
{
  m_nShowView = m_nShowView == LocalView ? FullView : LocalView;
  if (m_nShowView == LocalView) {
    m_pWdgShow[0]->show();
    m_pWdgShow[1]->show();
    m_pWdgFullView->hide();
  }
  else {
    m_pWdgShow[0]->hide();
    m_pWdgShow[1]->hide();
    m_pWdgFullView->show();
  }
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
  if (m_nShowView == LocalView) {
    m_pWdgShow[0]->setViewResolution(index);
    m_pWdgShow[1]->setViewResolution(index);
  }
  else {
    m_pWdgFullView->setViewResolution(index);
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
  m_pWdgFullView->clearMapDatas();
  m_bFlagPauseReplay = false;
  m_listPlanningFiles.clear();
  this->fileList(path.toStdString(), m_listPlanningFiles);
  m_pWdgParam->setFrameCount(m_listPlanningFiles.size());

  m_itFile = m_listPlanningFiles.begin();
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
  m_pWdgShow[1]->setShowAllTargets(show);
}

void QBaseWidget::onSelectTool(int index, bool checkable)
{
  m_pWdgShow[LivePlay]->setToolIndex(index, checkable);
  m_pWdgShow[RePlay]->setToolIndex(index, checkable);
}

