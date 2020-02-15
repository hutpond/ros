#include "qstateitem.h"

#include <QPainter>
#include <QSvgRenderer>
#include "GlobalDefine.h"

QStateItem::QStateItem(const QString &name, QWidget *parent)
  : QWidget(parent)
  , m_strName(name)
  , m_bFlagOn(false)
{

}

void QStateItem::setStateOn(bool flag)
{
  m_bFlagOn = flag;
  this->update();
}

void QStateItem::resizeEvent(QResizeEvent *)
{
  const int WIDTH = this->width();
  const int HEIGHT = this->height();
  if (WIDTH >= HEIGHT) {
    const int ITEM_W = static_cast<int>(HEIGHT * 0.8);
    m_rectSvg = QRect((WIDTH - ITEM_W) / 2, 0, ITEM_W, ITEM_W);
  }
  else {
    const int ITEM_W = static_cast<int>(WIDTH * 0.8);
    m_rectSvg = QRect(0, (HEIGHT - ITEM_W) / 2, ITEM_W, ITEM_W);
  }
}

void QStateItem::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  QSvgRenderer render;
  render.load(m_bFlagOn ? QStringLiteral(":image/state_on.svg") : QStringLiteral(":image/state_off.svg"));
  render.render(&painter, m_rectSvg);

  painter.setFont(G_TEXT_FONT);
  painter.drawText(m_rectSvg, Qt::AlignCenter, m_strName);
}
