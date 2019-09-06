/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningWidget.h
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划主界面，可以实时显示和回放
********************************************************/
#ifndef Q_PLANNING_WIDGET_H
#define Q_PLANNING_WIDGET_H

#include <boost/atomic.hpp>
#include <boost/filesystem.hpp>
#include <jsoncpp/json/json.h>
#include "QBaseWidget.h"
#include "QPlanningShowWidget.h"
#ifdef WIN64
# include "ReadDataManager.h"
#else
# include "QReadDataManagerRos.h"
#endif

class QPlanningShowWidget;
class QPlanningParamWidget;
struct PlanningData;

class QPlanningWidget : public QBaseWidget
{
  Q_OBJECT

public:
  QPlanningWidget(QWidget *parent);
  ~QPlanningWidget();

  void setViewResolution(int);
  void startReplay(int, const QString &);
  void stopDisplay();
  void setShowAllTargets(bool);
  void setShowIndex(int, bool);
  bool isIndexShow(int);
  void setReplaySpeedIndex(int);
  int replaySpeedIndex();

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void timerEvent(QTimerEvent *);
  virtual void showEvent(QShowEvent *);

protected:
  void replayJson(int, const QString &);
  bool readFromJsonFile(const std::string &, debug_tool::ads_PlanningData4Debug &);

  void sortTrackTargets(debug_tool::ads_PlanningData4Debug &);
  void saveDataToJsonFile(const debug_tool::ads_PlanningData4Debug &);
  void parseDataFromJson(const Json::Value &, debug_tool::ads_PlanningData4Debug &);

  void showWidgets();

protected slots:
  void onSetFrameIndexReplay(int, int);
  void onParsePlanningData(const debug_tool::ads_PlanningData4Debug &);
  void onSelectedShow();

private:
  QPlanningShowWidget *m_pWdgShow[3];
  QPlanningParamWidget *m_pWdgParam;

  bool m_bShowVisible[3];
  boost::filesystem::path m_fsPath;
  QRect m_rectShow;

  int m_nReplaySpeedIndex;
};

#endif // Q_PLANNING_WIDGET_H
