#include "QDecisionStateWidget.h"

#include "QFrameTimeWidget.h"

QDecisionStateWidget::QDecisionStateWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgFrameTime = new QFrameTimeWidget(this);
}

void QDecisionStateWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  float POS_X_PF = 0.02f;
  const float FRAME_W_PF = 0.5f;
  m_pWdgFrameTime->setGeometry(
        WIDTH * POS_X_PF,
        0,
        WIDTH * FRAME_W_PF,
        HEIGHT
        );
}

void QDecisionStateWidget::setPlanningData(quint64 preMillSecond, const debug_tool::ads_PlanningData4Debug &data)
{
  m_pWdgFrameTime->setPlanningData(preMillSecond, data);
}

void QDecisionStateWidget::clearData()
{
  m_pWdgFrameTime->clearData();
}
