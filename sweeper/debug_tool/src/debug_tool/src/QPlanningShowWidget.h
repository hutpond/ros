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

public:
  QPlanningShowWidget(QWidget *parent);
  ~QPlanningShowWidget();

  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);
  void setToolIndex(int, bool);

protected:
  void mousePressEvent(QMouseEvent *) final;
  void mouseMoveEvent(QMouseEvent *) final;

signals:

protected:
  void drawImage();
  void drawSweeper(QPainter &);
  void drawUltrasonic(QPainter &);
  void drawRadar(QPainter &);
  void drawRoadSide(QPainter &);
  void drawRoadSideFromWidth(QPainter &);
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
  void drawNewTarget(QPainter &);
  void drawText(QPainter &);

  void calcMapRect();

  int findReferenceIndex(const double s);
  int findReferenceIndex(const double x, const double y);
  void xyToSl(const QPointF &, double &s, double &l);
  void slToXy(const double s, const double l, QPointF &);
  QPolygonF createSlPgf(const QPointF &, double, double, bool = false);

  void addTracksToData();
  void addGarbageToData();

private:
  boost::array<QPointF, 100> m_ptfsLeftRoadSide;
  boost::array<QPointF, 100> m_ptfsRightRoadSide;

  debug_tool::ads_PlanningData4Debug  m_planningData;

  int m_nNewTracksCount;
  int m_nNewGarbageCount;
};

#endif // Q_PLANNING_SHOW_WIDGET_H
