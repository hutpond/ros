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
#include "QReadDataManagerRos.h"
#include "QPlanningCostWidget.h"

class QPlanningShowWidget;
class QPlanningParamWidget;
class QFullViewWidget;

class QPlanningWidget : public QBaseWidget
{
  Q_OBJECT

public:
  enum {
    LivePlay,
    RePlay
  };

  enum {
    LocalView,
    FullView
  };

public:
  QPlanningWidget(QWidget *parent);
  ~QPlanningWidget();

  void setViewResolution(int);
  void startReplay(const QString &);
  void stopDisplay();
  void setShowAllTargets(bool);
  void setReplaySpeedIndex(int);
  int replaySpeedIndex();
  void setShowType(int);
  void changeShowView();

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void timerEvent(QTimerEvent *);

protected:
  bool readFromJsonFile(const std::string &, debug_tool::ads_PlanningData4Debug &);

  void sortTrackTargets(debug_tool::ads_PlanningData4Debug &);
  void saveDataToJsonFile(const debug_tool::ads_PlanningData4Debug &);
  void parseDataFromJson(const Json::Value &, debug_tool::ads_PlanningData4Debug &);
  void setPlanningData(debug_tool::ads_PlanningData4Debug &, const QString &);

protected slots:
  void onSetFrameIndexReplay(int);
  void onParsePlanningData(const debug_tool::ads_PlanningData4Debug &);
  void onCostValueChanged();

private:
  QPlanningShowWidget *m_pWdgShow[2];
  QPlanningParamWidget *m_pWdgParam;
  QFullViewWidget *m_pWdgFullView;

  int m_nShowType;
  int m_nShowView;
  boost::filesystem::path m_fsPath;

  int m_nReplaySpeedIndex;

  double m_dCostValue[QPlanningCostWidget::Count];
};

#endif // Q_PLANNING_WIDGET_H
