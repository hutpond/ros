#ifndef QNEWPLANNINGSHOWWIDGET_H
#define QNEWPLANNINGSHOWWIDGET_H

#include "QBaseShowWidget.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QNewPlanningShowWidget : public QBaseShowWidget
{
  Q_OBJECT

  enum {
    EnuCoord,
    FrenetCoord
  };

public:
  explicit QNewPlanningShowWidget(QWidget *parent = Q_NULLPTR);

  void setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &);

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

private:
  debug_ads_msgs::ads_msgs_planning_debug_frame m_planningData;

  int m_nCoordType;
};

#endif // QNEWPLANNINGSHOWWIDGET_H
