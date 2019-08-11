/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningShowWidget.h
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划显示子画面，可以实时显示和回放
********************************************************/
#ifndef Q_PLANNING_SHOW_WIDGET_H
#define Q_PLANNING_SHOW_WIDGET_H

#include <boost/function.hpp>
#include "QBaseShowWidget.h"
#include "debug_tool/PlanningData4Debug.h"

class QPlanningShowWidget : public QBaseShowWidget
{
  Q_OBJECT

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

  void setPlanningData(const debug_tool::PlanningData4Debug &);
  void setFunPosition(boost::function<void(float, float, float, float)>);
  void setViewResolution(int);

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
  void drawPlanningPath(QPainter &);
  void drawObstacleObject(QPainter &);
  void drawUltrasonicTarget(QPainter &);
  void drawRadar28Target(QPainter &);
  void drawRadar73Target(QPainter &);
  void drawTrackTarget(QPainter &);
  void drawDecisionTargets(QPainter &);
  void drawDecisionTargetsSL(const debug_tool::TargetPoint_<std::allocator<void>> &, QPainter &);

  void calcMapRect();

  int findReferenceIndex(const double s);
  void xyToSl(const QPointF &, double &s, double &l);
  void slToXy(const double s, const double l, QPointF &);
  QPolygonF createSlPgf(const QPointF &, double, double, bool = false);

private:
  boost::function<void(float, float, float, float)> m_funPosition;
  boost::array<QPointF, 100> m_ptfsLeftRoadSide;
  boost::array<QPointF, 100> m_ptfsRightRoadSide;

  debug_tool::PlanningData4Debug  m_planningData;
  int m_nShowPlanningPath;  // 规划路线显示, 0: 全显示, 1：只显示当前帧, 2：只显示前一帧
};

#endif // Q_PLANNING_SHOW_WIDGET_H
