#include "qcentralwidget.h"
#include "qshowwidget.h"
#include "qstatewidget.h"
#include "decision_subscriber.h"

QCentralWidget::QCentralWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgShow = new QShowWidget(this);
  m_pWdgState = new QStateWidget(this);
  m_pObjSubscriber = new DecisionSubscriber(*this, this);
}

void QCentralWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const float WDG_STATE_H_PF = 0.13f;

  m_pWdgState->setGeometry(
        0, 0, WIDTH, HEIGHT * WDG_STATE_H_PF
        );

  m_pWdgShow->setGeometry(
        0, HEIGHT * WDG_STATE_H_PF, WIDTH, HEIGHT * (1.0 - WDG_STATE_H_PF)
        );
}

void QCentralWidget::setData(const decision_studio::ads_DecisionData4Debug &data)
{
  m_pWdgShow->setData(data);
  m_pWdgState->setData(data);
}
