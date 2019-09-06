#ifndef QPLANNINGCOSTWIDGET_H
#define QPLANNINGCOSTWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QPlotWidget;

class QPlanningCostWidget : public QWidget
{
  Q_OBJECT

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
  void resizeEvent(QResizeEvent *) final;

signals:

public slots:

private:
  QPlotWidget *m_pWdgCost[Count];
};

#endif // QPLANNINGCOSTWIDGET_H
