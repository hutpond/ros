#ifndef QPLANNINGCOSTWIDGET_H
#define QPLANNINGCOSTWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QwtPlot;
class QwtPlotCurve;

class QPlanningCostWidget : public QWidget
{
  Q_OBJECT

public:
  enum
  {
    Cost,
    Safety,
    Lateral,
    Smoothness,
    Consistency,
    Garbage,
    Count
  };

public:
  explicit QPlanningCostWidget(QWidget *parent = nullptr);
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *) final;
  virtual void showEvent(QShowEvent *) final;

signals:

public slots:

private:
  QwtPlot *m_pWdgPlotCostNew;
  QwtPlotCurve *m_pPlotCurveCostNew;

  QwtPlot *m_pWdgPlotCost[Count];
  QwtPlotCurve *m_pPlotCurveCost[Count];
};

#endif // QPLANNINGCOSTWIDGET_H
