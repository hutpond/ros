#include "qstatewidget.h"
#include "qstateitem.h"

QStateWidget::QStateWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgItem[StateForward] = new QStateItem(QStringLiteral("前进"), this);
  m_pWdgItem[StateFollow] = new QStateItem(QStringLiteral("跟车"), this);
  m_pWdgItem[StatePass] = new QStateItem(QStringLiteral("绕行"), this);
  m_pWdgItem[StateSafeStop] = new QStateItem(QStringLiteral("停车"), this);
}

void QStateWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const int SPACE_Y = qMin<int>(HEIGHT * 0.1, 3);
  const int ITEM_H = HEIGHT - 2 * SPACE_Y;
  const int ITEM_W = ITEM_H * 0.8;
  const float SPACE_X = (float)(WIDTH - ITEM_W * StateCount) / (float)(StateCount + 1);

  for (int i = 0; i < StateCount; ++i) {
    m_pWdgItem[i]->setGeometry(
          SPACE_X + i * (SPACE_X + ITEM_W),
          SPACE_Y, ITEM_W, ITEM_H
          );
  }
}

void QStateWidget::setData(const decision_studio::ads_DecisionData4Debug &data)
{

}
