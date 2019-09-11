#include "QPlanningCostWidget.h"
#include "QPlotWidget.h"
#include "QCostValueWidget.h"

QPlanningCostWidget::QPlanningCostWidget(QWidget *parent) : QWidget(parent)
{
  m_pWdgCostNew = new QPlotWidget(this);
  m_pWdgCostNew->setName("cost new", "index", "cost");
  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i] = new QPlotWidget(this);
  }
  m_pWdgCost[Cost]->setName("cost", "index", "cost");
  m_pWdgCost[Safety]->setName("safety", "index", "cost");
  m_pWdgCost[Lateral]->setName("lateral", "index", "cost");
  m_pWdgCost[Smoothness]->setName("smoothness", "index", "cost");
  m_pWdgCost[Consistency]->setName("consistency", "index", "cost");
  m_pWdgCost[Garbage]->setName("garbage", "index", "cost");
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
  m_pWdgCostNew->setGeometry(pos_x, space_y, item_w, item_h);
  for (int i = 0; i < Count; ++i) {
    pos_x += (space_x + item_w);
    m_pWdgCost[i]->setGeometry(pos_x, space_y, item_w, item_h);
  }
}

void QPlanningCostWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  m_pWdgCostNew->clearPoints();
  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i]->clearPoints();
  }
  const auto &candidates = data.planning_trajectory_candidates;
  const int size_candidates = static_cast<int>(candidates.size());
  double value[Count];
  QCostValueWidget::getCostValue(value);

  QSharedPointer<QPointF> ptr_ptf;
  for (int i = 0; i < size_candidates; ++i) {
    QPointF *ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Cost]->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    double cost_new = value[Safety] * candidates[i].safety_cost
        + value[Lateral] * candidates[i].lateral_cost
        + value[Smoothness] * candidates[i].smoothness_cost
        + value[Consistency] * candidates[i].consistency_cost
        + value[Garbage] * candidates[i].garbage_cost;
    ptf->setY(cost_new);
    ptr_ptf.reset(ptf);
    m_pWdgCostNew->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].safety_cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Safety]->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].lateral_cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Lateral]->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].smoothness_cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Smoothness]->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].consistency_cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Consistency]->addPoint(ptr_ptf);

    ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].garbage_cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Garbage]->addPoint(ptr_ptf);
  }

  m_pWdgCostNew->plot();
  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i]->plot();
  }
}
