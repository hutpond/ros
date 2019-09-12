/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningShowWidget.h
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划显示子画面，可以实时显示和回放
********************************************************/
#ifndef Q_PLANNING_SHOW_WIDGET_H
#define Q_PLANNING_SHOW_WIDGET_H

#include "QBaseShowWidget.h"
#include "debug_tool/ads_PlanningData4Debug.h"

class QPlanningShowWidget : public QBaseShowWidget
{
  Q_OBJECT

public:

  enum {
    SENSOR_UNKNOW = 0,
    SENSOR_ULTRASONIC,
    SENSOR_RADAR_28F,
    SENSER_RADAR_73F,
    SENSOR_TRACK_TARGET,
    SENSOR_LIDAR,
    SENSOR_CAMERA
  };

  enum {
    NEW_COST,
    OLD_COST
  };

public:
  QPlanningShowWidget(QWidget *parent);
  ~QPlanningShowWidget();

  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);
  void setShowAllTargets(bool);
  void setCostType(int);

protected:
  virtual void mousePressEvent(QMouseEvent *);

protected:
  void drawImage();
  void drawSweeper(QPainter &);
  void drawUltrasonic(QPainter &);
  void drawRadar(QPainter &);
  void drawRoadSide(QPainter &);
  void drawAreaLine(QPainter &);
  void drawDecisionPoint(QPainter &);
  void drawPlanningPoint(QPainter &);
  void drawObstacleObject(QPainter &);
  void drawUltrasonicTarget(QPainter &);
  void drawRadar28Target(QPainter &);
  void drawRadar73Target(QPainter &);
  void drawTrackTarget(QPainter &);
  void drawTrackTargetWithPoints(QPainter &);
  void drawDecisionTargets(QPainter &);
  void drawDecisionTargetsSL(const debug_tool::ads_TargetPoint_<std::allocator<void>> &, QPainter &);
  void drawPlanningSplines(QPainter &);
  void drawPlanningCandidatesSplines(QPainter &);
  void drawGarbageResults(QPainter &);
  void drawBezierLine(QPainter &, const std::vector< ::debug_tool::ads_Spline_<std::allocator<void>>> &);

  void calcMapRect();

  int findReferenceIndex(const double s);
  void xyToSl(const QPointF &, double &s, double &l);
  void slToXy(const double s, const double l, QPointF &);
  QPolygonF createSlPgf(const QPointF &, double, double, bool = false);

private:
  boost::array<QPointF, 100> m_ptfsLeftRoadSide;
  boost::array<QPointF, 100> m_ptfsRightRoadSide;

  debug_tool::ads_PlanningData4Debug  m_planningData;
  int m_nShowPlanningPath;  // 规划路线显示, 0: 全显示, 1：只显示当前帧, 2：只显示前一帧
  bool m_bFlagShowAllTargets;
  int m_nCostType;
};

#endif // Q_PLANNING_SHOW_WIDGET_H
