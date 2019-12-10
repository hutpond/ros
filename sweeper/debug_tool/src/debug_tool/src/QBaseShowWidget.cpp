/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseShowWidget.h
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 图像显示基类
********************************************************/
#include "QBaseShowWidget.h"

#include <cmath>
#include <QMouseEvent>
#include <QPainter>
#include "GlobalDefine.h"

QBaseShowWidget::QBaseShowWidget(QWidget *parent)
  : QWidget(parent)
  , m_fDisplayRatio(1.0)
  , m_fOriginRatio(1.0)
  , m_nCostType(OLD_COST)
  , m_bFlagShowAllTargets(false)
{
}

QBaseShowWidget::~QBaseShowWidget()
{
}

void QBaseShowWidget::showEvent(QShowEvent *)
{
  this->doUpdate(false);
}

void QBaseShowWidget::resizeEvent(QResizeEvent *)
{
  m_rectPicture = this->rect();
  this->doUpdate(false);
}

void QBaseShowWidget::mouseMoveEvent(QMouseEvent *e)
{
  if ((Qt::LeftButton & e->buttons()) == Qt::NoButton) {
    return;
  }
  QPointF ptf = e->localPos();
  QLineF linef(m_ptfMouseMove, ptf);
  m_ptfMouseMove = ptf;
  QTransform inverted = m_transform.inverted();
  linef = inverted.map(linef);
  m_ptfTranslate += (linef.p2() - linef.p1());

  this->doUpdate(true);
}

void QBaseShowWidget::wheelEvent(QWheelEvent *e)
{
  const float RATIO_COEF = 1.05f;
  int delta = e->delta() / 120;
  m_fDisplayRatio /= std::pow(RATIO_COEF, delta);
  this->doUpdate(true);
}

void QBaseShowWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.drawImage(0, 0, m_image);
}

/*******************************************************
 * @brief 设置显示鼠标点处物理坐标的回调函数
 * @param fun: 回调函数

 * @return
********************************************************/
void QBaseShowWidget::setFunPosition(boost::function<void(float, float, float, float)> fun)
{
  m_funPosition = fun;
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
      m_fDisplayRatio = m_fOriginRatio;
      m_ptfTranslate = QPointF(0, 0);
      break;
    case 1:
      m_fDisplayRatio /= RATIO_COEF;
      break;
    default:
      break;
  }
  this->doUpdate(true);
}

void QBaseShowWidget::doUpdate(bool update)
{
  if (this->isVisible()) {
    this->calcMapRect();
    this->drawImage();
    if (update) {
      this->update();
    }
  }
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
  pen.setBrush(Qt::gray);
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

void QBaseShowWidget::setCostType(int type)
{
  m_nCostType = type;
}

/*******************************************************
 * @brief 是否显示所有target
 * @param show: true, 显示所有

 * @return
********************************************************/
void QBaseShowWidget::setShowAllTargets(bool show)
{
  m_bFlagShowAllTargets = show;
  this->doUpdate(true);
}


QPointF QBaseShowWidget::pixelToMap(const QPointF &ptfPixel)
{
  QTransform inverted = m_transform.inverted();
  QPointF ptfMap = inverted.map(ptfPixel);
  return ptfMap;
}

void QBaseShowWidget::addTargetMouseMove(QMouseEvent *e)
{
  m_ptfTargetMove = e->localPos();
  const auto size = m_ptfTargets.size();
  if (size == 1 || size == 2) {
    this->doUpdate(true);
  }
}

QPolygonF QBaseShowWidget::createTargetPgf(const QVector<QPointF> &ptfs, const QPointF &ptfMove)
{
  QPolygonF pgf;
  if (ptfs.size() != 2) {
    return pgf;
  }
  QLineF linef(ptfs[0], ptfs[1]);
  QLineF linef2;
  linef2.setP1(ptfMove);
  linef2.setAngle(linef.angle());

  QLineF linefNormal;

  linefNormal.setP1(ptfs[0]);
  linefNormal.setAngle(linef.normalVector().angle());
  QPointF ptfIntersect;
  linefNormal.intersect(linef2, &ptfIntersect);
  pgf << ptfs[0] << ptfIntersect;

  linefNormal.setP1(ptfs[1]);
  linefNormal.setAngle(linef.normalVector().angle());
  linefNormal.intersect(linef2, &ptfIntersect);
  pgf << ptfIntersect << ptfs[1] << ptfs[0];

  return pgf;
}

