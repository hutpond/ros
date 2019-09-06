/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseWidget.h
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 子模块基类
********************************************************/
#ifndef Q_BASE_WIDGET_H
#define Q_BASE_WIDGET_H

#include <QWidget>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>
#include <boost/atomic.hpp>
#include "jsoncpp/json/json.h"

class QDataDisplayDialog;
class QBaseWidget : public QWidget
{
  Q_OBJECT

public:
  QBaseWidget(QWidget *parent);
  ~QBaseWidget();

protected:
  void replay(int);
  std::list<std::string> pathList(const std::string &);
  void fileList(const std::string &, std::vector<std::string> &);

protected slots:
  void setReplayInterval(int);
  void onReplayState(int, bool);

protected:
  boost::filesystem::path m_fspath[2];
//  boost::atomic_int m_nIntervalMillSecs; // replay interval mill seconds
  boost::atomic_bool m_bFlagPauseReplay[2]; // replay pause state;
  std::vector<std::string> m_listPlanningFiles[2];
  std::vector<std::string>::iterator m_itFile[2];  // 文件名链表迭代器
  int m_nTimerId;         // replay定时器id
};

#endif // Q_BASE_WIDGET_H
