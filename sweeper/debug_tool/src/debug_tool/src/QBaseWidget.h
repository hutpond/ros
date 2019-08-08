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
  enum
  {
    TypeNone = -1,
    LiveDisplay,
    Replay
  };

public:
  QBaseWidget(QWidget *parent);
  ~QBaseWidget();

protected:
  void replay(const std::string &);
  std::list<std::string> pathList(const std::string &);
  std::list<std::string> fileList(const std::string &);

protected slots:
  void setReplayInterval(int);
  void onReplayState(bool);

protected:
  boost::atomic_int m_nShowType;  // 0: live display; 1: replay

  boost::filesystem::path m_fspath;
  boost::atomic_int m_nIntervalMillSecs; // replay interval mill seconds
  boost::atomic_bool m_bFlagPauseReplay; // replay pause state;
  std::list<std::string> m_listPlanningFiles;
  std::list<std::string>::iterator m_itFile;  // 文件名链表迭代器
  int m_nTimerId;         // replay定时器id

  QDataDisplayDialog *m_pDlgDataDisplay;
};

#endif // Q_BASE_WIDGET_H
