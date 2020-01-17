#include "qstateitem.h"

#include <QPainter>
#include <QSvgRenderer>

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
  const int SPACE = qMin<int>(HEIGHT * 0.05, 3);

  const float SVG_H_PF = 0.58;
  const int SVG_W = qMin<int>(HEIGHT * SVG_H_PF, WIDTH - 2 * SPACE);
  m_rectSvg = QRect((WIDTH - SVG_W) / 2, SPACE, SVG_W, SVG_W);

  m_rectName = QRect(0, SPACE * 2 + SVG_W, WIDTH, HEIGHT - 3 * SPACE - SVG_W);
}

void QStateItem::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  QSvgRenderer render;
  render.load(m_bFlagOn ? QStringLiteral(":image/state_on.svg") : QStringLiteral(":image/state_off.svg"));
  render.render(&painter, m_rectSvg);

  painter.setFont(QFont("SimHei", 14));
  painter.drawText(m_rectName, Qt::AlignCenter, m_strName);
}
