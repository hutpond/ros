#include "qstatewidget.h"
#include "qstateitem.h"

QStateWidget::QStateWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pWdgItem[StateForward] = new QStateItem(QStringLiteral("前进"), this);
  m_pWdgItem[StateLeftPass] = new QStateItem(QStringLiteral("左侧\n绕行"), this);
  m_pWdgItem[StateRightPass] = new QStateItem(QStringLiteral("右侧\n绕行"), this);
  m_pWdgItem[StateFollow] = new QStateItem(QStringLiteral("跟随"), this);
  m_pWdgItem[StateWait] = new QStateItem(QStringLiteral("停车\n等待"), this);
  m_pWdgItem[StateSafeStop] = new QStateItem(QStringLiteral("安全\n停车"), this);
  m_pWdgItem[StateExitAuto] = new QStateItem(QStringLiteral("退出"), this);

  m_pWdgItem[StateSafeStop]->setStateOn(true);
}

void QStateWidget::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();

  const int SPACE_X = 2;
  const float ITEM_W = float(WIDTH - SPACE_X * (StateCount + 1))  / StateCount;

  for (int i = 0; i < StateCount; ++i) {
    m_pWdgItem[i]->setGeometry(
          SPACE_X + i * (SPACE_X + ITEM_W),
          0, ITEM_W, HEIGHT
          );
  }
}

void QStateWidget::setData(const debug_tool::ads_PlanningData4Debug &data)
{
  int index = static_cast<int>(data.planning_output.decision);
  if (index >= 0 && index < StateCount) {
    for (int i = 0; i < StateCount; ++i) {
      m_pWdgItem[i]->setStateOn(i == index);
    }
  }
}
