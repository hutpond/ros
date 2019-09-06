#include "QPlanningCostWidget.h"
#include "QPlotWidget.h"

QPlanningCostWidget::QPlanningCostWidget(QWidget *parent) : QWidget(parent)
{
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
  const int item_w = double(width - space_x * (Count + 1)) / double(Count);
  const int item_h = height - 2 * space_y;
  space_x = double(width - item_w * Count) / double(Count + 1);
  space_y = (height - item_h) / 2.0;

  float pos_x = space_x;
  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i]->setGeometry(pos_x, space_y, item_w, item_h);

    pos_x += (space_x + item_w);
  }
}

void QPlanningCostWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i]->clearPoints();
  }
  const auto &candidates = data.planning_trajectory_candidates;
  const int size_candidates = static_cast<int>(candidates.size());

  QSharedPointer<QPointF> ptr_ptf;
  for (int i = 0; i < size_candidates; ++i) {
    QPointF *ptf = new QPointF;
    ptf->setX(i);
    ptf->setY(candidates[i].cost);
    ptr_ptf.reset(ptf);
    m_pWdgCost[Cost]->addPoint(ptr_ptf);

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

  for (int i = 0; i < Count; ++i) {
    m_pWdgCost[i]->plot();
  }
}
