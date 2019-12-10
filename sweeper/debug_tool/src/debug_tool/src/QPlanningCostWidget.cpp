#include <algorithm>
#include <cmath>

#include "QPlanningCostWidget.h"
#include "QCostValueWidget.h"
#include "GlobalDefine.h"

#include "qwt_plot.h"
#include "qwt_plot_curve.h"
#include "qwt_plot_grid.h"

QPlanningCostWidget::QPlanningCostWidget(QWidget *parent) : QWidget(parent)
{
  m_pWdgPlotCostNew = new QwtPlot(QStringLiteral("cost new"), this);
//  m_pWdgPlotCostNew->setAxisTitle(QwtPlot::xBottom, QStringLiteral("index"));
//  m_pWdgPlotCostNew->setAxisTitle(QwtPlot::yLeft, QStringLiteral("cost"));
  QwtPlotGrid *grid = new QwtPlotGrid;
  grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
  grid->attach(m_pWdgPlotCostNew);

  m_pWdgPlotCostNew->setAxisFont(QwtPlot::xBottom, QFont("Times", 10));
  m_pWdgPlotCostNew->setAxisFont(QwtPlot::yLeft, QFont("Times", 10));

  m_pPlotCurveCostNew = new QwtPlotCurve( "Curve 1" );
  m_pPlotCurveCostNew->attach(m_pWdgPlotCostNew);

  m_pWdgPlotCost[Cost] = new QwtPlot(QStringLiteral("cost"), this);
  m_pWdgPlotCost[Safety] = new QwtPlot(QStringLiteral("safety"), this);
  m_pWdgPlotCost[Lateral] = new QwtPlot(QStringLiteral("lateral"), this);
  m_pWdgPlotCost[Smoothness] = new QwtPlot(QStringLiteral("smoothness"), this);
  m_pWdgPlotCost[Consistency] = new QwtPlot(QStringLiteral("consistency"), this);
  m_pWdgPlotCost[Garbage] = new QwtPlot(QStringLiteral("garbage"), this);

  for (int i = 0; i < Count; ++i) {
    m_pWdgPlotCost[i]->setAxisFont(QwtPlot::xBottom, QFont("Times", 10));
    m_pWdgPlotCost[i]->setAxisFont(QwtPlot::yLeft, QFont("Times", 10));
    grid = new QwtPlotGrid;
    grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->attach(m_pWdgPlotCost[i]);

//    m_pWdgPlotCost[i]->setAxisTitle(QwtPlot::xBottom, QStringLiteral("index"));
//    m_pWdgPlotCost[i]->setAxisTitle(QwtPlot::yLeft, QStringLiteral("cost"));

    m_pPlotCurveCost[i] = new QwtPlotCurve( "Curve 1" );
    m_pPlotCurveCost[i]->attach(m_pWdgPlotCost[i]);
  }
}

void QPlanningCostWidget::resizeEvent(QResizeEvent *)
{
  const int width = this->width();
  const int height = this->height();

  float space_x = 5;
  float space_y = 5;
  const int item_w = double(width - space_x * (Count + 2)) / double(Count + 1);
  const int item_h = height - 2 * space_y;
  space_x = double(width - item_w * (Count + 1)) / double(Count + 2);
  space_y = (height - item_h) / 2.0;

  float pos_x = space_x;
  m_pWdgPlotCostNew->setGeometry(pos_x, space_y, item_w, item_h);
  for (int i = 0; i < Count; ++i) {
    pos_x += (space_x + item_w);
    m_pWdgPlotCost[i]->setGeometry(pos_x, space_y, item_w, item_h);
  }
}

void QPlanningCostWidget::showEvent(QShowEvent *)
{
  m_pWdgPlotCostNew->replot();
  for (int i = 0; i < Count; ++i) {
    m_pWdgPlotCost[i]->replot();
  }
}

void QPlanningCostWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  QVector<QPointF> ptfNew;
  QVector<QPointF> pointfs[Count];
  double value[Count];
  QCostValueWidget::getCostValue(value);

  auto candidates = data.planning_trajectory_candidates;
  if (candidates.size() == 0) {
    return;
  }
  using type_candidates = decltype(candidates[0]);
  std::sort(candidates.begin(),  candidates.end(), [](
            const type_candidates &item, const type_candidates &item2) {
    return item.id > item2.id;
  });

  const int size_candidates = static_cast<int>(candidates.size());
  for (int i = 0; i < size_candidates; ++i) {
    double cost_new = value[Safety] * candidates[i].safety_cost
        + value[Lateral] * candidates[i].lateral_cost
        + value[Smoothness] * candidates[i].smoothness_cost
        + value[Consistency] * candidates[i].consistency_cost
        + value[Garbage] * candidates[i].garbage_cost;
    ptfNew.push_back(QPointF(candidates[i].id, cost_new));

    pointfs[Cost].push_back(QPointF(candidates[i].id, candidates[i].cost));
    pointfs[Safety].push_back(QPointF(candidates[i].id, candidates[i].safety_cost));
    pointfs[Lateral].push_back(QPointF(candidates[i].id, candidates[i].lateral_cost));
    pointfs[Smoothness].push_back(QPointF(candidates[i].id, candidates[i].smoothness_cost));
    pointfs[Consistency].push_back(QPointF(candidates[i].id, candidates[i].consistency_cost));
    pointfs[Garbage].push_back(QPointF(candidates[i].id, candidates[i].garbage_cost));
  }

  int xMax = candidates[0].id;
  int xMin = candidates[size_candidates - 1].id;
  xMax = (xMax % 10 == 0 ? xMax : (xMax / 10 * 10 + 10));
  xMin = (xMax % 10 == 0 ? xMin : (xMin / 10 * 10 + 10));

  m_pWdgPlotCostNew->setAxisScale(QwtPlot::xBottom, xMax, xMin, -10);
  m_pPlotCurveCostNew->setSamples(ptfNew);
  if (this->isVisible()) {    
    m_pWdgPlotCostNew->replot();
  }
  for (int i = 0; i < Count; ++i) {
    m_pWdgPlotCost[i]->setAxisScale(QwtPlot::xBottom, xMax, xMin, -10);
    m_pPlotCurveCost[i]->setSamples(pointfs[i]);
    if (this->isVisible()) {
      m_pWdgPlotCost[i]->replot();
    }
  }
}
