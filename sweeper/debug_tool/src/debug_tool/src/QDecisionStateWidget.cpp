#include "QDecisionStateWidget.h"

#include "qstatewidget.h"
#include "QFrameTimeWidget.h"

QDecisionStateWidget::QDecisionStateWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgState = new QStateWidget(this);
  m_pWdgFrameTime = new QFrameTimeWidget(this);
}

void QDecisionStateWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const float STATE_W_PF = 0.2;
  const float STATE_H_PF = 0.5;
  m_pWdgState->setGeometry(
        0,
        HEIGHT * (1.0f - STATE_H_PF) / 2.0f,
        WIDTH * STATE_W_PF,
        HEIGHT * STATE_H_PF
        );

  float POS_X_PF = STATE_W_PF + 0.02f;
  const float FRAME_W_PF = 0.5f;
  m_pWdgFrameTime->setGeometry(
        WIDTH * POS_X_PF,
        0,
        WIDTH * FRAME_W_PF,
        HEIGHT
        );
}

void QDecisionStateWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  m_pWdgState->setData(data);
  m_pWdgFrameTime->setPlanningData(data);
}

void QDecisionStateWidget::clearData()
{
  m_pWdgFrameTime->clearData();
}
