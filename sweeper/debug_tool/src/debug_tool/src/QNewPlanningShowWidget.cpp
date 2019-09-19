#include <QPainter>
#include <QMouseEvent>
#include "QNewPlanningShowWidget.h"
#include "QEditToolsWidget.h"
#include "GlobalDefine.h"

QNewPlanningShowWidget::QNewPlanningShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
  , m_nCoordType(EnuCoord)
{

}

void QNewPlanningShowWidget::mousePressEvent(QMouseEvent *e)
{
  bool bLeftPress = (e->buttons() & Qt::LeftButton);

  QPointF ptf = e->localPos();
  if (bLeftPress) {
    QPointF ptfMap = this->pixelToMap(ptf);
    double s, l;
    m_funPosition(ptfMap.x(), ptfMap.y(), s, l);
  }

  if ( m_nToolIndex == QEditToolsWidget::Target ||
       m_nToolIndex == QEditToolsWidget::Garbage ) {
    if (bLeftPress) {
      if (m_ptfTargets.size() < 2) {
        m_ptfTargets << ptf;
      }
      else if (m_ptfTargets.size() == 2){
        if (m_nToolIndex == QEditToolsWidget::Target) {
          //this->addTracksToData();
        }
        else if (m_nToolIndex == QEditToolsWidget::Garbage) {
          //this->addGarbageToData();
        }
        this->doUpdate(true);
      }
    }
    else {
      m_ptfTargets.clear();
      this->doUpdate(true);
    }
  }
  else {
    m_ptfMouseMove = ptf;
  }
}

void QNewPlanningShowWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));
  this->drawMapBorder(painter);
  this->drawAxis(painter);

  if (m_planningData.reference_line_enu.size() == 0) {
    return;
  }
  this->drawVehicle(painter);
  this->drawReference(painter);
  this->drawObstacle(painter);
  this->drawTrajectory(painter);
}

void QNewPlanningShowWidget::calcMapRect()
{
  const double height = 25.0 * m_fDisplayRatio;
  const double width = height * m_rectPicture.width() / m_rectPicture.height();

  double vehicle_x_min = 1000;
  double vehicle_x_max = -1000;
  double vehicle_y_min = 1000;
  for (int i = 0; i < 4; ++i) {
    if (vehicle_y_min > m_planningData.ego_state_enu[i].Y) {
      vehicle_y_min = m_planningData.ego_state_enu[i].Y;
    }
    if (vehicle_x_min > m_planningData.ego_state_enu[i].X) {
      vehicle_x_min = m_planningData.ego_state_enu[i].X;
    }
    if (vehicle_x_max < m_planningData.ego_state_enu[i].X) {
      vehicle_x_max = m_planningData.ego_state_enu[i].X;
    }
  }
  double pos_x = (vehicle_x_max + vehicle_x_min) / 2.0 - width / 2.0 - m_ptfTranslate.x();
  double pos_y = vehicle_y_min - 1.0 - m_ptfTranslate.y();
  m_rectfMap = QRectF(pos_x, pos_y, width, height);

  // 坐标转换
  m_transform.reset();
  m_transform.rotate(180, Qt::XAxis);
  m_transform.scale(m_rectPicture.width() / m_rectfMap.width(),
                    m_rectPicture.height() / m_rectfMap.height());
  m_transform.translate(-m_rectfMap.x(), -(m_rectfMap.y() + m_rectfMap.height()));
}

void QNewPlanningShowWidget::setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  m_ptfTargets.clear();

  m_planningData = data;
  this->doUpdate(true);
}

void QNewPlanningShowWidget::drawAxis(QPainter &painter)
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
  polygonX << ptX << (ptX + QPoint(-25, 3)) << (ptX + QPoint(-25, -3));
  painter.drawPolygon(polygonX);
  painter.drawText(
        lineX.p2() + QPoint(-19, 12),
        m_nCoordType == EnuCoord ?"X" : "l"
        );

  // draw line arrow text
  QLineF lineY = QLineF(0, m_rectfMap.y(), 0, m_rectfMap.height() + m_rectfMap.y());
  lineY = m_transform.map(lineY);
  painter.drawLine(lineY);
  QPoint ptY = lineY.p2().toPoint();
  QPolygon polygonY;
  polygonY << ptY << (ptY + QPoint(3, 35)) << (ptY + QPoint(-3, 35));
  painter.drawPolygon(polygonY);
  painter.drawText(
        lineY.p2() + QPoint(8, 19),
        m_nCoordType == EnuCoord ?"Y" : "s"
        );
  painter.restore();
}

void QNewPlanningShowWidget::drawReference(QPainter &painter)
{
  QPolygonF pgf;
  if (m_nCoordType == EnuCoord) {
    for (const auto &point : m_planningData.reference_line_enu) {
      pgf << QPointF(point.X, point.Y);
    }
  }
  else {
    for (const auto &point : m_planningData.reference_line_frenet) {
      pgf << QPointF(point.l, point.s);
    }
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::black);
    pen.setStyle(Qt::DashLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    painter.restore();
  }
}

void QNewPlanningShowWidget::drawVehicle(QPainter &painter)
{
  QPolygonF pgf;
  if (m_nCoordType == EnuCoord) {
    for (const auto &point : m_planningData.ego_state_enu) {
      pgf << QPointF(point.X, point.Y);
    }
  }
  else {
    for (const auto &point : m_planningData.ego_state_frenet) {
      pgf << QPointF(point.l, point.s);
    }
  }

  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    painter.restore();
  }
}

void QNewPlanningShowWidget::drawTrajectory(QPainter &painter)
{
  // current
  /*QPolygonF pgf;
  for (const auto &point : m_planningData.trajectories) {
    pgf << QPointF(point.X, point.Y);
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(1);
    pen.setColor(Qt::black);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    painter.restore();
  }*/
}

void QNewPlanningShowWidget::drawObstacle(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setWidth(2);
  pen.setColor(Qt::darkMagenta);
  pen.setStyle(Qt::SolidLine);
  painter.setFont(QFont("Times", 10));
  painter.setPen(pen);

  for (const auto &obstack : m_planningData.obstacles) {
    QPolygonF pgf;
    if (m_nCoordType == EnuCoord) {
      for (const auto &point : obstack.points_enu) {
        pgf << QPointF(point.X, point.Y);
      }
    }
    else {
      for (const auto &point : obstack.points_frenet) {
        pgf << QPointF(point.l, point.s);
      }
    }
    if (pgf.size() > 0) {
      pgf = m_transform.map(pgf);
      painter.drawPolyline(pgf);
    }
  }
  painter.restore();
}

