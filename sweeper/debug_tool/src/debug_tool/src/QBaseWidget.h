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

class QBaseShowWidget;
class QPlanningParamWidget;
class QFullViewWidget;
class QNewPlanningPlot;

class QBaseWidget : public QWidget
{
  Q_OBJECT

public:
  enum {
    LivePlay,
    RePlay
  };

  enum {
    LocalViewVehicle,
    LocalViewENU,
    LocalViewFrenet,
    PlottingView,
    FullView
  };

public:
  QBaseWidget(QWidget *parent);
  ~QBaseWidget();

  void setReplaySpeedIndex(int);
  int replaySpeedIndex();

  void setShowType(int);
  int showType();
  virtual void changeShowView() = 0;
  int showView();

  void setViewResolution(int);
  void startReplay(const QString &);
  void stopDisplay();
  void setShowAllTargets(bool);

protected:
  virtual void resizeEvent(QResizeEvent *);

protected:
  void fileList(const std::string &, std::vector<std::string> &);

  std::string dataFileName();
  std::string dataFileName(long, long);

public slots:
  void onSelectTool(int, bool);

protected slots:
  void onReplayState(bool);

protected:
  QBaseShowWidget *m_pWdgShow[2];
  QNewPlanningPlot *m_pWdgPlotting;
  QPlanningParamWidget *m_pWdgParam;
  QFullViewWidget *m_pWdgFullView;

  int m_bFlagPauseReplay; // replay pause state;
  std::vector<std::string> m_listPlanningFiles;
  std::vector<std::string>::iterator m_itFile;  // 文件名链表迭代器
  int m_nTimerId;         // replay定时器id

  int m_nShowType;
  int m_nShowView;
  boost::filesystem::path m_fsPath;
  std::string m_strJsonFile;

  int m_nReplaySpeedIndex;
};

#endif // Q_BASE_WIDGET_H
