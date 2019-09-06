/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseShowWidget.h
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 图像显示基类
********************************************************/
#include <cmath>
#include <QMouseEvent>
#include <QPainter>
#include "QBaseShowWidget.h"
#include "GlobalDefine.h"

QBaseShowWidget::QBaseShowWidget(QWidget *parent)
  : QWidget(parent)
  , m_fDisplayRatio(MAP_TO_ROAD_COEF)
{
}

QBaseShowWidget::~QBaseShowWidget()
{
}

void QBaseShowWidget::resizeEvent(QResizeEvent *)
{
  m_rectPicture = this->rect();
  this->calcMapRect();
  this->drawImage();
}

void QBaseShowWidget::mouseMoveEvent(QMouseEvent *e)
{
  QPointF ptf = e->localPos();
  QLineF linef(m_ptfMouseMove, ptf);
  m_ptfMouseMove = ptf;
  QTransform inverted = m_transform.inverted();
  linef = inverted.map(linef);
  m_ptfTranslate += (linef.p2() - linef.p1());

  this->calcMapRect();
  this->drawImage();
  this->update();
}

void QBaseShowWidget::wheelEvent(QWheelEvent *e)
{
  const float RATIO_COEF = 1.05f;
  int delta = e->delta() / 120;
  m_fDisplayRatio /= std::pow(RATIO_COEF, delta);
  this->calcMapRect();
  this->drawImage();
  this->update();
}

void QBaseShowWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.drawImage(0, 0, m_image);
}

/*******************************************************
 * @brief 缩放图像显示
 * @param index: -1， 缩小, 0, 复原, 1, 放大

 * @return
********************************************************/
void QBaseShowWidget::setViewResolution(int index)
{
  const float RATIO_COEF = 1.2;
  switch (index) {
    case -1:
      m_fDisplayRatio *= RATIO_COEF;
      break;
    case 0:
      m_fDisplayRatio = MAP_TO_ROAD_COEF;
      m_ptfTranslate = QPointF(0, 0);
      break;
    case 1:
      m_fDisplayRatio /= RATIO_COEF;
      break;
    default:
      break;
  }
  this->calcMapRect();
  this->drawImage();
  this->update();
}

/*******************************************************
 * @brief 绘制画图区域边框
 * @param painter: 画笔

 * @return
********************************************************/
void QBaseShowWidget::drawMapBorder(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setWidth(4);
  pen.setColor(Qt::red);
  pen.setBrush(m_bFlagSelected ? Qt::green : Qt::gray);
  painter.setPen(pen);
  QRectF border(
        m_rectfMap.topLeft(),
        m_rectfMap.size()
        );
  QPolygonF pgfBorder = m_transform.map(border);
  painter.drawPolygon(pgfBorder);
  painter.restore();
}

/*******************************************************
 * @brief 绘制坐标轴
 * @param painter: 画笔

 * @return
********************************************************/
void QBaseShowWidget::drawAxis(QPainter &painter)
{
  painter.save();
  painter.setPen(Qt::black);
  painter.setBrush(Qt::black);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setFont(G_TEXT_SMALL_FONT);

  // draw line arrow text
  QLineF lineX = QLineF(m_rectfMap.x(), 0, m_rectfMap.width() + m_rectfMap.x(), 0);
  lineX = m_transform.map(lineX);
  painter.drawLine(lineX);
  QPoint ptX = lineX.p2().toPoint();
  QPolygon polygonX;
  polygonX << ptX << (ptX + QPoint(3, 25)) << (ptX + QPoint(-3, 25));
  painter.drawPolygon(polygonX);
  painter.drawText(
        lineX.p2() + QPoint(8, 19),
        "X"
        );

  // draw line arrow text
  QLineF lineY = QLineF(0, m_rectfMap.y(), 0, m_rectfMap.height() + m_rectfMap.y());
  lineY = m_transform.map(lineY);
  painter.drawLine(lineY);
  QPoint ptY = lineY.p2().toPoint();
  QPolygon polygonY;
  polygonY << ptY << (ptY + QPoint(35, 3)) << (ptY + QPoint(35, -3));
  painter.drawPolygon(polygonY);
  painter.drawText(
        lineY.p2() + QPoint(19, -8),
        "Y"
        );
  painter.restore();
}
