#include "QNewPlanningShowWidget.h"

#include <QPainter>
#include <QMouseEvent>
#include "QEditToolsWidget.h"
#include "GlobalDefine.h"
#include "QDebugToolMainWnd.h"
#include "QDataDisplayWidget.h"

QNewPlanningShowWidget::QNewPlanningShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
{
  m_nCoordType = EnuCoord;
  m_nToolIndex = QEditToolsWidget::Move;
}

void QNewPlanningShowWidget::mousePressEvent(QMouseEvent *e)
{
  bool bLeftPress = (e->buttons() & Qt::LeftButton);

  QPointF ptf = e->localPos();
  if (bLeftPress) {
    QPointF ptfMap = this->pixelToMap(ptf);
    if (m_nCoordType == EnuCoord) {
      m_funPosition(ptfMap.x(), ptfMap.y(), 0, 0);
    }
    else {
      m_funPosition(0, 0, ptfMap.x(), ptfMap.y());
    }
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
  if (m_nCoordType == EnuCoord) {
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
  else {
    const double width = 25.0 * m_fDisplayRatio;
    const double height = width * m_rectPicture.width() / m_rectPicture.height();

    double vehicle_l_min = 1000;
    double vehicle_l_max = -1000;
    double vehicle_s_min = 1000;
    for (int i = 0; i < 4; ++i) {
      if (vehicle_s_min > m_planningData.ego_state_frenet[i].s) {
        vehicle_s_min = m_planningData.ego_state_frenet[i].s;
      }
      if (vehicle_l_min > m_planningData.ego_state_frenet[i].l) {
        vehicle_l_min = m_planningData.ego_state_frenet[i].l;
      }
      if (vehicle_l_max < m_planningData.ego_state_frenet[i].l) {
        vehicle_l_max = m_planningData.ego_state_frenet[i].l;
      }
    }
    double pos_l = (vehicle_l_max + vehicle_l_min) / 2.0 - height / 2.0 - m_ptfTranslate.y();
    double pos_s = vehicle_s_min - 1.0 - m_ptfTranslate.x();
    m_rectfMap = QRectF(pos_s, pos_l, width, height);

    // 坐标转换
    m_transform.reset();
    m_transform.rotate(90);
    m_transform.rotate(180, Qt::YAxis);
    m_transform.scale(m_rectPicture.height() / m_rectfMap.width(),
                      m_rectPicture.width() / m_rectfMap.height());
    m_transform.translate(-(m_rectfMap.x() + m_rectfMap.width()),
                          -(m_rectfMap.y() + m_rectfMap.height()));
  }
}

void QNewPlanningShowWidget::setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &data)
{
  m_ptfTargets.clear();

  m_planningData = data;
  QDebugToolMainWnd::s_pDataDisplay->setPlanningData(data);
  this->doUpdate(true);
}

void QNewPlanningShowWidget::setShowCoord(int coord)
{
  if (m_nCoordType != coord) {
    m_nCoordType = coord;
    this->doUpdate(true);
  }
}

void QNewPlanningShowWidget::drawAxis(QPainter &painter)
{
  painter.save();
  painter.setPen(Qt::black);
  painter.setBrush(Qt::black);
  painter.setRenderHint(QPainter::Antialiasing, true);
  painter.setFont(G_TEXT_SMALL_FONT);

  // draw line arrow text
  if (m_nCoordType == EnuCoord) {
    QLineF lineX = QLineF(m_rectfMap.x(), 0, m_rectfMap.width() + m_rectfMap.x(), 0);
    lineX = m_transform.map(lineX);
    painter.drawLine(lineX);
    QPoint ptX = lineX.p2().toPoint();
    QPolygon polygonX;
    polygonX << ptX << (ptX + QPoint(-25, 3)) << (ptX + QPoint(-25, -3));
    painter.drawPolygon(polygonX);
    painter.drawText(lineX.p2() + QPoint(-19, 12), "X");

    // draw line arrow text
    QLineF lineY = QLineF(0, m_rectfMap.y(), 0, m_rectfMap.height() + m_rectfMap.y());
    lineY = m_transform.map(lineY);
    painter.drawLine(lineY);
    QPoint ptY = lineY.p2().toPoint();
    QPolygon polygonY;
    polygonY << ptY << (ptY + QPoint(3, 35)) << (ptY + QPoint(-3, 35));
    painter.drawPolygon(polygonY);
    painter.drawText(lineY.p2() + QPoint(8, 19), "Y" );
  }
  else {
    QLineF line_s = QLineF(m_rectfMap.x(), 0, m_rectfMap.width() + m_rectfMap.x(), 0);
    line_s = m_transform.map(line_s);
    painter.drawLine(line_s);
    QPoint pt_s = line_s.p2().toPoint();
    QPolygon polygonX;
    polygonX << pt_s << (pt_s + QPoint(3, 35)) << (pt_s + QPoint(-3, 35));
    painter.drawPolygon(polygonX);
    painter.drawText(line_s.p2() + QPoint(8, 19), "S");

    // draw line arrow text
    QLineF line_l = QLineF(0, m_rectfMap.y(), 0, m_rectfMap.height() + m_rectfMap.y());
    line_l = m_transform.map(line_l);
    painter.drawLine(line_l);
    QPoint pt_l = line_l.p2().toPoint();
    QPolygon polygonY;
    polygonY << pt_l << (pt_l + QPoint(35, 3)) << (pt_l + QPoint(35, -3));
    painter.drawPolygon(polygonY);
    painter.drawText(line_l.p2() + QPoint(19, -8), "L" );
  }
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
    this->addPointsToPolygenF(pgf, m_planningData.ego_state_enu);
  }
  else {
    this->addPointsToPolygenF(pgf, m_planningData.ego_state_frenet);
  }

  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(2);
    pen.setColor(Qt::red);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolygon(pgf);

    painter.restore();
  }
}

void QNewPlanningShowWidget::drawTrajectory(QPainter &painter)
{
  // current
  QPolygonF pgf;
  if (m_nCoordType == EnuCoord) {
    for (const auto &point : m_planningData.trajectory_current.current_traj_points_enu) {
      pgf << QPointF(point.X, point.Y);
    }
  }
  else {
    for (const auto &point : m_planningData.trajectory_current.current_traj_points_frenet) {
      pgf << QPointF(point.s, point.l);
    }
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(2);
    pen.setColor(Qt::blue);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    painter.restore();
  }

  // last
  pgf.clear();
  if (m_nCoordType == EnuCoord) {
    for (const auto &point : m_planningData.trajectory_last.current_traj_points_enu) {
      pgf << QPointF(point.X, point.Y);
    }
  }
  else {
    for (const auto &point : m_planningData.trajectory_last.current_traj_points_frenet) {
      pgf << QPointF(point.s, point.l);
    }
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();
    QPen pen;
    pen.setWidth(2);
    pen.setColor(Qt::magenta);
    pen.setStyle(Qt::SolidLine);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    painter.restore();
  }
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

  int index = 0;
  for (const auto &obstack : m_planningData.obstacles) {
    QPolygonF pgf;
    if (m_nCoordType == EnuCoord) {
      this->addPointsToPolygenF(pgf, obstack.points_enu);
    }
    else {
      this->addPointsToPolygenF(pgf, obstack.points_frenet);
    }
    if (pgf.size() > 0) {
      pgf = m_transform.map(pgf);
      painter.drawPolygon(pgf);

      painter.drawText(pgf.boundingRect().center(), QString::number(index ++));
    }
  }
  painter.restore();
}

void QNewPlanningShowWidget::addPointsToPolygenF(
    QPolygonF &pgf,
    const boost::array<debug_ads_msgs::ads_msgs_planning_debug_pointENU, 4> &points)
{
  QLineF linef(points[0].X, points[0].Y, points[1].X, points[1].Y);
  QLineF linef2(points[2].X, points[2].Y, points[3].X, points[3].Y);
  QPointF ptf;
  if (linef.intersect(linef2, &ptf) == QLineF::BoundedIntersection) {
    pgf << QPointF(points[0].X, points[0].Y)
        << QPointF(points[2].X, points[2].Y)
        << QPointF(points[1].X, points[1].Y)
        << QPointF(points[3].X, points[3].Y);
  }
  else {
    pgf << QPointF(points[0].X, points[0].Y)
        << QPointF(points[1].X, points[1].Y)
        << QPointF(points[2].X, points[2].Y)
        << QPointF(points[3].X, points[3].Y);
  }
}

void QNewPlanningShowWidget::addPointsToPolygenF(
    QPolygonF &pgf,
    const boost::array<debug_ads_msgs::ads_msgs_planning_debug_pointFRENET, 4> &points)
{
  QLineF linef(points[0].s, points[0].l, points[1].s, points[1].l);
  QLineF linef2(points[2].s, points[2].l, points[3].s, points[3].l);
  QPointF ptf;
  if (linef.intersect(linef2, &ptf) == QLineF::BoundedIntersection) {
    pgf << QPointF(points[0].s, points[0].l)
        << QPointF(points[2].s, points[2].l)
        << QPointF(points[1].s, points[1].l)
        << QPointF(points[3].s, points[3].l);
  }
  else {
    pgf << QPointF(points[0].s, points[0].l)
        << QPointF(points[1].s, points[1].l)
        << QPointF(points[2].s, points[2].l)
        << QPointF(points[3].s, points[3].l);
  }
}

