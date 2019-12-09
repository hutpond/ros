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
#include "debug_tool/ads_PlanningData4Debug.h"
#include "QPlanningCostWidget.h"

class QPlanningShowWidget;
class QPlanningParamWidget;
class QFullViewWidget;

class QPlanningWidget : public QBaseWidget
{
  Q_OBJECT

public:
  QPlanningWidget(QWidget *parent);
  ~QPlanningWidget();

  void changeShowView() final;

public slots:
  void onSaveDataToFile(const debug_tool::ads_PlanningData4Debug &);

protected:
  virtual void timerEvent(QTimerEvent *);

protected:
  bool readFromJsonFile(const std::string &, debug_tool::ads_PlanningData4Debug &);

  void sortTrackTargets(debug_tool::ads_PlanningData4Debug &);
  void saveDataToJsonFile(const std::string &, const debug_tool::ads_PlanningData4Debug &);
  void parseDataFromJson(const Json::Value &, debug_tool::ads_PlanningData4Debug &);
  void setPlanningData(debug_tool::ads_PlanningData4Debug &, const QString &);

  debug_tool::ads_PlanningData4Debug calcPlanningPathWitCost(const debug_tool::ads_PlanningData4Debug &);
  bool ObstacleCollisionCheck(const debug_tool::ads_planning_trajectory &,
      const debug_tool::ads_TrackTarget &, double, double, double = 0.0);
  bool RoadBoundaryCheck(const debug_tool::ads_planning_trajectory &,
      const std::vector< ::debug_tool::ads_Point > &,
      double, double = 0.0);

protected slots:
  void onSetFrameIndexReplay(int);
  void onParsePlanningData(const debug_tool::ads_PlanningData4Debug &);
  void onCostValueChanged();

private:

  double m_dCostValue[QPlanningCostWidget::Count];
};

#endif // Q_PLANNING_WIDGET_H
