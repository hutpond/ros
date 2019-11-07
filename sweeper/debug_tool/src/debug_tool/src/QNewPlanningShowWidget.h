#ifndef QNEWPLANNINGSHOWWIDGET_H
#define QNEWPLANNINGSHOWWIDGET_H

#include "QBaseShowWidget.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QNewPlanningShowWidget : public QBaseShowWidget
{
  Q_OBJECT

public:
  explicit QNewPlanningShowWidget(QWidget *parent = Q_NULLPTR);

  void setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &);
  virtual void setShowCoord(int) final;

protected:
  void mousePressEvent(QMouseEvent *);

protected:
  void drawImage();
  void calcMapRect();

  void drawAxis(QPainter &);
  void drawReference(QPainter &);
  void drawVehicle(QPainter &);
  void drawTrajectory(QPainter &);
  void drawObstacle(QPainter &);

  void addPointsToPolygenF(
      QPolygonF &,
      const boost::array<debug_ads_msgs::ads_msgs_planning_debug_pointENU, 4> &);
  void addPointsToPolygenF(
      QPolygonF &,
      const boost::array<debug_ads_msgs::ads_msgs_planning_debug_pointFRENET, 4> &);

private:
  debug_ads_msgs::ads_msgs_planning_debug_frame m_planningData;
};

#endif // QNEWPLANNINGSHOWWIDGET_H
