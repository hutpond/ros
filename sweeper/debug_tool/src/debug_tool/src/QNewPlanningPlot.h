#ifndef QNEWPLANNINGPLOT_H
#define QNEWPLANNINGPLOT_H

#include <QWidget>
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QwtPlot;
class QwtPlotCurve;

class QNewPlanningPlot : public QWidget
{
  Q_OBJECT

  enum
  {
    PlotVT,
    PlotAT,
    PlotYawT,
    PlotKT,
    PlotST,
    PlotXT,
    PlotYT,
    PlotYX,
    PlotCount
  };

public:
  explicit QNewPlanningPlot(QWidget *parent = nullptr);

  void setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &);

signals:

public slots:

private:
  QwtPlot *m_pWdgPlot[PlotCount];
  QwtPlotCurve *m_pPlotCurve[PlotCount];
};

#endif // QNEWPLANNINGPLOT_H
