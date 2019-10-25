/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDrawPathWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/19
 * Description: 路径绘制
********************************************************/
#include <cfloat>
#include <QTimerEvent>
#include <QPainter>
#include <QTimerEvent>
#include <QMouseEvent>
#include <QApplication>
#include "QDrawPathWidget.h"
#include "QProjectObject.h"
#include "QPanelWidget.h"

QDrawPathWidget::QDrawPathWidget(QProjectObject &project, QWidget *parent)
  : QWidget(parent)
  , m_rObjProject(project)
  , m_rectfSelect(0, 0, 0, 0)
  , m_rectfSelectMap(0, 0, 0, 0)
  , m_ptfTranslate(0, 0)
  , m_nDisplayRatio(1)
  , m_nOperateIndex(QPanelWidget::TypeMove)
  , m_bMouseLeftPressed(false)
  , m_enumMapOperation(MapOperation::None)
{
  m_nTimerId = startTimer(1000);
}

QDrawPathWidget::~QDrawPathWidget()
{
  if (m_nTimerId != 0) {
    killTimer(m_nTimerId);
    m_nTimerId = 0;
  }
}

void QDrawPathWidget::resizeEvent(QResizeEvent *)
{
  m_rectPicture = this->rect();
}

void QDrawPathWidget::paintEvent(QPaintEvent *)
{
  QPainter painter(this);
  painter.drawImage(0, 0, m_image);
}

void QDrawPathWidget::timerEvent(QTimerEvent *e)
{
  int id = e->timerId();
  if (id == m_nTimerId) {
    this->doUpdate();
  }
}

void QDrawPathWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));

#ifdef DEBUG
  this->drawMapBorder(painter);
#endif
  this->drawLane(painter);
  if (m_nOperateIndex == QPanelWidget::TypeZoomLocal && m_bMouseLeftPressed) {
    this->drawSelectRect(painter);
  }
}

/*******************************************************
 * @brief 绘制画图区域边框
 * @param painter: 画笔

 * @return
********************************************************/
void QDrawPathWidget::drawMapBorder(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setWidth(4);
  pen.setColor(Qt::red);
  pen.setBrush(Qt::green);
  painter.setPen(pen);
  QRectF border(
        m_rectfMap.topLeft(),
        m_rectfMap.size()
        );
  QPolygonF pgfBorder = m_transform.map(border);
  painter.drawPolygon(pgfBorder);
  painter.restore();
}

void QDrawPathWidget::drawLane(QPainter &painter)
{
  const apollo::hdmap::Map &map = m_rObjProject.mapData();
  const int size_lane = map.lane_size();

  painter.save();
  for (int i = 0; i < size_lane; ++i) {
    const apollo::hdmap::Lane &lane = map.lane(i);

    // central curve
    if (i == 0) {
      const int size_segment = lane.central_curve().segment_size();
      if (size_segment == 0) {
        continue;
      }
      const auto &curve = lane.central_curve();
      QPolygonF &&pgf = this->getCurvePoints(curve);
      if (pgf.size() > 0) {
        QPen pen;
        pen.setColor(Qt::black);
        pen.setWidth(1);
        pen.setStyle(Qt::DashDotLine);
        painter.setPen(pen);

        pgf = m_transform.map(pgf);
        painter.drawPolyline(pgf);
      }
    }

    // left boundary
    {
      const auto curve = lane.left_boundary().curve();
      QPolygonF &&pgf = this->getCurvePoints(curve);
      if (pgf.size() > 0) {
        QPen pen;
        pen.setColor(Qt::yellow);
        pen.setWidth(2);
        pen.setStyle(Qt::SolidLine);
        painter.setPen(pen);

        pgf = m_transform.map(pgf);
        painter.drawPolyline(pgf);
      }
    }

    // right boundary
    {
      const auto curve = lane.right_boundary().curve();
      QPolygonF &&pgf = this->getCurvePoints(curve);
      if (pgf.size() > 0) {
        QPen pen;
        pen.setColor(Qt::yellow);
        pen.setWidth(2);
        pen.setStyle(Qt::SolidLine);
        painter.setPen(pen);

        pgf = m_transform.map(pgf);
        painter.drawPolyline(pgf);
      }
    }
  }
  painter.restore();
}

void QDrawPathWidget::drawSelectRect(QPainter &painter)
{
  if (qFuzzyCompare(m_rectfSelect.width(), 0)) return;
  painter.save();
  QPen pen;
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawRect(m_rectfSelect);
  painter.restore();
}

void QDrawPathWidget::doUpdate()
{
  this->calcMapRect();
  this->drawImage();
  this->update();
}

void QDrawPathWidget::calcMapRect()
{
  constexpr float COEF = 1.2;
  if (qFuzzyCompare(m_rectfSelectMap.width(), 0)) {
    const apollo::hdmap::Map &map = m_rObjProject.mapData();
    int size = map.lane_size();
    if (size > 0) {
      const apollo::hdmap::Lane &lane = map.lane(0);
      size = lane.central_curve().segment_size();
      if (size > 0) {
        const auto &curve_segment = lane.central_curve().segment(0);
        const auto &line_segment = curve_segment.line_segment();
        size = line_segment.point_size();
      }
    }

    if (size == 0) {
      const float WIDTH = 8;
      const float LENGTH = WIDTH / m_rectPicture.width() * m_rectPicture.height();
      m_rectfMap = QRectF(0, 0, WIDTH, LENGTH);
    }
    else {
      const apollo::hdmap::Lane &lane = map.lane(0);
      const auto &curve_segment = lane.central_curve().segment(0);
      const auto &line_segment = curve_segment.line_segment();

      float x_min = MAX_POS;
      float x_max = -MAX_POS;
      float y_min = MAX_POS;
      float y_max = -MAX_POS;
      for (int i = 0; i < size; ++i) {
        const auto &pt = line_segment.point(i);
        if (x_min > pt.x()) {
          x_min = pt.x();
        }
        if (x_max < pt.x()) {
          x_max = pt.x();
        }
        if (y_min > pt.y()) {
          y_min = pt.y();
        }
        if (y_max < pt.y()) {
          y_max = pt.y();
        }
      }
      float width = x_max - x_min;
      float height = y_max - y_min;
      double ratio = (double)m_rectPicture.width() / (double)m_rectPicture.height();
      if ( width / height > ratio) {
        height = width * m_rectPicture.height() / m_rectPicture.width();
      }
      else {
        width = height * m_rectPicture.width() / m_rectPicture.height();
      }
      width *= qPow(COEF, m_nDisplayRatio);
      height *= qPow(COEF, m_nDisplayRatio);
      x_min -= (width - (x_max - x_min)) / 2.0 + m_ptfTranslate.x();
      y_min -= (height - (y_max - y_min)) / 2.0 + m_ptfTranslate.y();

      m_rectfMap = QRectF(x_min, y_min, width, height);
    }
  }
  else {
    float width = m_rectfSelectMap.width();
    float height = m_rectfSelectMap.height();
    double ratio = (double)m_rectPicture.width() / (double)m_rectPicture.height();
    if ( width / height > ratio) {
      height = width * m_rectPicture.height() / m_rectPicture.width();
    }
    else {
      width = height * m_rectPicture.width() / m_rectPicture.height();
    }
    width *= qPow(COEF, m_nDisplayRatio);
    height *= qPow(COEF, m_nDisplayRatio);
    m_rectfMap = QRectF(
          m_rectfSelectMap.x() - m_ptfTranslate.x(),
          m_rectfSelectMap.y() - m_ptfTranslate.y(),
          width,
          height
          );
  }

  // 坐标转换
  m_transform.reset();
  m_transform.rotate(180, Qt::XAxis);
  m_transform.scale(m_rectPicture.width() / m_rectfMap.width(),
                    m_rectPicture.height() / m_rectfMap.height());
  m_transform.translate(-m_rectfMap.x(), -(m_rectfMap.y() + m_rectfMap.height()));
}

void QDrawPathWidget::onOperate(int index)
{
  switch (index) {
    case QPanelWidget::TypeMove:
    case QPanelWidget::TypeZoomLocal:
      m_nOperateIndex = index;
      QApplication::restoreOverrideCursor();
      break;
    case QPanelWidget::TypeDelete:
      {
        m_nOperateIndex = index;
        QPixmap pixmap(":/image/mouse.svg");
        m_nDeleteRadius = 20;
        pixmap = pixmap.scaled(m_nDeleteRadius, m_nDeleteRadius);
        QCursor cursor(pixmap);
        QApplication::setOverrideCursor(cursor);
      }
      break;
    case QPanelWidget::TypeZoomIn:
      m_nDisplayRatio--;
      this->doUpdate();
      break;
    case QPanelWidget::TypeZoomOut:
      m_nDisplayRatio++;
      this->doUpdate();
      break;
    case QPanelWidget::TypeZoomReset:
      m_nDisplayRatio = 1;
      m_ptfTranslate = QPointF(0, 0);
      m_rectfSelectMap.setWidth(0);
      this->doUpdate();
      break;
    default:
      break;
  }
}

void QDrawPathWidget::onAddBoundary(int index_lane, int index_segment, int index_side)
{
  m_enumMapOperation = MapOperation::Boundary;

  m_nCurrentLane = index_lane;
  m_nCurrentSegment = index_segment;
  m_nBoundarySide = index_side;
}

void QDrawPathWidget::onOperateSignal(MapOperation operation)
{
  m_enumMapOperation = operation;
}

void QDrawPathWidget::mousePressEvent(QMouseEvent *e)
{
  if (e->button() == Qt::LeftButton) {
    m_ptfMouseMove = e->localPos();
    m_bMouseLeftPressed = true;
    this->addElementToMap(m_ptfMouseMove);
  }
}

void QDrawPathWidget::mouseReleaseEvent(QMouseEvent *e)
{
  if (e->button() == Qt::LeftButton && m_nOperateIndex == QPanelWidget::TypeZoomLocal) {
    m_bMouseLeftPressed = false;
    QPointF ptfTopLeft = m_rectfSelect.topLeft();
    QPointF ptfBottomRight = m_rectfSelect.bottomRight();
    QTransform inverted = m_transform.inverted();
    ptfTopLeft = inverted.map(ptfTopLeft);
    ptfBottomRight = inverted.map(ptfBottomRight);
    m_rectfSelectMap = QRectF(
          qMin<qreal>(ptfTopLeft.x(), ptfBottomRight.x()),
          qMin<qreal>(ptfTopLeft.y(), ptfBottomRight.y()),
          qAbs<qreal>(ptfTopLeft.x() - ptfBottomRight.x()),
          qAbs<qreal>(ptfTopLeft.y() - ptfBottomRight.y())
        );
    m_rectfSelect.setWidth(0);

    this->doUpdate();
  }
}

void QDrawPathWidget::mouseMoveEvent(QMouseEvent *e)
{
  if (!m_bMouseLeftPressed) {
    return;
  }
  QPointF ptf = e->localPos();

  if (m_nOperateIndex == QPanelWidget::TypeMove) {
    QLineF linef(m_ptfMouseMove, ptf);
    m_ptfMouseMove = ptf;
    QTransform inverted = m_transform.inverted();
    linef = inverted.map(linef);
    m_ptfTranslate += (linef.p2() - linef.p1());
  }
  else if (m_nOperateIndex == QPanelWidget::TypeDelete) {
    if (m_bMouseLeftPressed) {
      /*QTransform inverted = m_transform.inverted();
      QLineF linef = QLineF(ptf.x(), ptf.y(), ptf.x() + m_nDeleteRadius / 2 - 3, ptf.y());
      linef = inverted.map(linef);
      const double RADIUS = linef.length();
      ptf = inverted.map(ptf);
      QList<QSharedPointer<MapBinData>> points;
      foreach (auto &pt, points) {
        if (pt->clear) {
          continue;
        }
        double length = QLineF(ptf.x(), ptf.y(), pt->x, pt->y).length();
        if (length <= RADIUS) {
          pt->clear = true;
        }
      }*/
    }
  }
  else if (m_nOperateIndex == QPanelWidget::TypeZoomLocal) {
    m_rectfSelect = QRectF(
          qMin<qreal>(m_ptfMouseMove.x(), ptf.x()),
          qMin<qreal>(m_ptfMouseMove.y(), ptf.y()),
          qAbs<qreal>(m_ptfMouseMove.x() - ptf.x()),
          qAbs<qreal>(m_ptfMouseMove.y() - ptf.y())
        );
  }

  this->doUpdate();
}

void QDrawPathWidget::wheelEvent(QWheelEvent *e)
{
  m_nDisplayRatio += e->delta() / 120;
  this->doUpdate();
}

void QDrawPathWidget::addElementToMap(const QPointF &ptf)
{
  switch (m_enumMapOperation) {
    case MapOperation::None:
      break;
    case MapOperation::Boundary:
      this->addBoundaryPoint(ptf);
      break;
    case MapOperation::SignalSign:
      this->addSignalSign(ptf);
      m_enumMapOperation = MapOperation::None;
      break;
    case MapOperation::Crosswalk:
      this->addCrosswalk(ptf);
      m_enumMapOperation = MapOperation::None;
      break;
    case MapOperation::StopSign:
      m_enumMapOperation = MapOperation::None;
      break;
    case MapOperation::YieldSign:
      m_enumMapOperation = MapOperation::None;
      break;
    case MapOperation::ClearArea:
      m_enumMapOperation = MapOperation::None;
      break;
    case MapOperation::SpeedBump:
      m_enumMapOperation = MapOperation::None;
      break;
  }
  this->doUpdate();
}

void QDrawPathWidget::addBoundaryPoint(const QPointF &ptf)
{
  apollo::hdmap::Map &map = m_rObjProject.mapData();
  auto lane = map.mutable_lane(m_nCurrentLane);
  if (lane == nullptr) {
    return;
  }
  apollo::hdmap::LaneBoundary *boundary = nullptr;
  if (m_nBoundarySide == static_cast<int>(BoundarySide::Left)) {
    boundary = lane->mutable_left_boundary();
  }
  else {
    boundary = lane->mutable_right_boundary();
  }
  if (boundary == nullptr) {
    return;
  }

  auto curve_segment = boundary->mutable_curve();
  if (curve_segment == nullptr) {
    return;
  }
  const int size_boundary_segment = curve_segment->segment_size();
  if (m_nCurrentSegment >= size_boundary_segment) {
    return;
  }
  auto line_segment = curve_segment->mutable_segment(m_nCurrentSegment)->mutable_line_segment();

  QPointF ptfENU = this->pixelToENU(ptf);
  auto pointENU = line_segment->add_point();
  pointENU->set_x(ptfENU.x());
  pointENU->set_y(ptfENU.y());
}

void QDrawPathWidget::addSignalSign(const QPointF &ptf)
{

}

void QDrawPathWidget::addCrosswalk(const QPointF &ptf)
{
  QPointF ptfENU = this->pixelToENU(ptf);

  apollo::hdmap::Map &map = m_rObjProject.mapData();
  auto crosswalk = map.add_crosswalk();
  const int size = map.crosswalk_size();

  crosswalk->mutable_id()->set_id("crosswalk_" + std::to_string(size + 1));
  apollo::hdmap::Polygon* polygon = crosswalk->mutable_polygon();
  apollo::common::PointENU* pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170000.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170000.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(170003.0);
  pt->set_y(170003.0);
  pt->set_z(0.0);
  pt = polygon->add_point();
  pt->set_x(170000.0);
  pt->set_y(170003.0);
  pt->set_z(0.0);
}

QPointF QDrawPathWidget::pixelToENU(const QPointF &ptf)
{
  QTransform inverted = m_transform.inverted();
  QPointF ptfENU = inverted.map(ptf);
  return ptfENU;
}

QPolygonF QDrawPathWidget::getCurvePoints(const apollo::hdmap::Curve &curve)
{
  QPolygonF pgf;
  const int size = curve.segment_size();
  for (int i = 0; i < size; ++i) {
    const auto &line_segment = curve.segment(i).line_segment();
    const int size_pts = line_segment.point_size();
    for (int j = 0; j < size_pts; ++j) {
      const auto &pt = line_segment.point(j);
      pgf << QPointF(pt.x(), pt.y());
    }
  }
  return std::move(pgf);
}

