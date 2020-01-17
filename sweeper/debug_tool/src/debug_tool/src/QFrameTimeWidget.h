#ifndef QFRAMETIMEWIDGET_H
#define QFRAMETIMEWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QwtPlot;
class QwtPlotCurve;

class QFrameTimeWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QFrameTimeWidget(QWidget *parent = nullptr);

  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);
  void clearData();

protected:
  virtual void resizeEvent(QResizeEvent *) final;
  virtual void showEvent(QShowEvent *) final;

private:
  QwtPlot *m_pWdgPlot;
  QwtPlotCurve *m_pPlotCurve;
  QVector<QPointF> m_points;
  double m_dCurrentMS;
};

#endif // QFRAMETIMEWIDGET_H
