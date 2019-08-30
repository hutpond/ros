/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 子模块基类
********************************************************/
#include <sstream>
#include "QBaseWidget.h"

QBaseWidget::QBaseWidget(QWidget *parent)
  : QWidget(parent)
  , m_nShowType(TypeNone)
  , m_nTimerId(0)
{
  m_pDlgDataDisplay = NULL;
}

QBaseWidget::~QBaseWidget()
{
}


/*******************************************************
 * @brief 重放数据文件
 * @param subdir: 子文件夹名称

 * @return
********************************************************/
void QBaseWidget::replay(const std::string &)
{
  std::string path = m_fspath.string();

  m_nIntervalMillSecs = 300;
  m_bFlagPauseReplay = false;
  m_listPlanningFiles.clear();
  this->fileList(path, m_listPlanningFiles);

  m_itFile = m_listPlanningFiles.begin();
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
  m_bFlagPauseReplay = false;
  m_nTimerId = startTimer(m_nIntervalMillSecs);
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
 * @brief 设置replay帧间隔时间
 * @param ms: 帧间隔时间，毫秒

 * @return
********************************************************/
void QBaseWidget::setReplayInterval(int ms)
{
  m_nIntervalMillSecs = ms;
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
