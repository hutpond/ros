/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningShowWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划显示子画面，可以实时显示和回放
********************************************************/
#include "QPlanningShowWidget.h"

#include <cmath>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QThread>
#include <QtMath>
#include "GlobalDefine.h"
#include "QCostValueWidget.h"
#include "QEditToolsWidget.h"
#include "QBezierCurve.h"

QPlanningShowWidget::QPlanningShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
  , m_nNewTracksCount(0)
  , m_nNewGarbageCount(0)
{
  m_nCoordType = VehicleCoord;
  m_fOriginRatio = 4.0;
  m_fDisplayRatio = m_fOriginRatio;
  this->setMouseTracking(true);
}

QPlanningShowWidget::~QPlanningShowWidget()
{
}

void QPlanningShowWidget::mousePressEvent(QMouseEvent *e)
{
  bool bLeftPress = (e->buttons() & Qt::LeftButton);

  QPointF ptf = e->localPos();
  if (bLeftPress) {
    QPointF ptfMap = this->pixelToMap(ptf);
    double s = 0, l = 0;
    const int size_reference_points = m_planningData.reference_points.size();
    if (size_reference_points > 0) {
      this->xyToSl(ptfMap, s, l);
    }
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
          this->addTracksToData();
        }
        else if (m_nToolIndex == QEditToolsWidget::Garbage) {
          this->addGarbageToData();
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

void QPlanningShowWidget::mouseMoveEvent(QMouseEvent *e)
{
  if (m_nToolIndex == QEditToolsWidget::Move) {
    QBaseShowWidget::mouseMoveEvent(e);
  }
  else if ( m_nToolIndex == QEditToolsWidget::Target ||
            m_nToolIndex == QEditToolsWidget::Garbage ) {
    this->addTargetMouseMove(e);
  }
}

/*******************************************************
 * @brief 设置扫规划数据
 * @param data: 规划数据

 * @return
********************************************************/
void QPlanningShowWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  m_nNewTracksCount = 0;
  m_nNewGarbageCount = 0;
  m_ptfTargets.clear();

  m_planningData = data;

  this->doUpdate(true);
}

/*******************************************************
 * @brief 计算显示区域，车体坐标系

 * @return
********************************************************/
void QPlanningShowWidget::calcMapRect()
{
  const int size_reference_splines = m_planningData.reference_splines.size();
  double left_road_width = 5;
  double right_road_width = 3;
  if (size_reference_splines == 0) {
    left_road_width = 5;
    right_road_width = 3;
    m_planningData.front_vehicle_length = 2.2;
  }
  else {
    left_road_width = m_planningData.reference_points[0].left_road_width;
    right_road_width = m_planningData.reference_points[0].right_road_width;
  }

  // 计算显示区域物理范围，车体坐标系，X正向：上，Y正向：左，坐标原点：车中心
  // 显示范围，height（Y向）：路宽MAP_TO_ROAD_COEF倍，
  // width（X向）：根据显示区域比例计算，起点：车身后START_X_TO_CAR_TAIL米
  const float VEH_HEAD = m_planningData.head_point.x;
  const float roadLeftWidth = left_road_width;
  const float roadRightWidth = right_road_width;
  const float roadWidth = roadLeftWidth + roadRightWidth;
  const float mapHeight = m_fDisplayRatio * roadWidth;
  const float mapWidth = mapHeight * m_rectPicture.height() / m_rectPicture.width();
  const float mapX = -(m_planningData.front_vehicle_length - VEH_HEAD)
      - m_ptfTranslate.x();
  const float mapY = -(roadRightWidth + roadWidth * (m_fDisplayRatio - 1.0) / 2.0)
      - m_ptfTranslate.y();
  m_rectfMap = QRectF(mapX, mapY, mapWidth, mapHeight);

  // 坐标转换
  m_transform.reset();
  m_transform.rotate(90);
  m_transform.rotate(180, Qt::YAxis);
  m_transform.scale(m_rectPicture.height() / m_rectfMap.width(),
                    m_rectPicture.width() / m_rectfMap.height());
  m_transform.translate(-(m_rectfMap.x() + m_rectfMap.width()),
                        -(m_rectfMap.y() + m_rectfMap.height()));
}

void QPlanningShowWidget::setToolIndex(int index, bool checkable)
{
  if (checkable) {
    m_nToolIndex = index;
  }
  if ( index == QEditToolsWidget::Save &&
       (m_nNewTracksCount > 0 || m_nNewGarbageCount > 0) ) {
    m_nNewTracksCount = 0;
    m_nNewGarbageCount = 0;
    emit saveDataToFile(m_planningData);

    this->doUpdate(true);
  }
}

/*******************************************************
 * @brief 根据数据将图像画在QImage上

 * @return
********************************************************/
void QPlanningShowWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));
  this->drawMapBorder(painter);
  this->drawAxis(painter);
  if (m_planningData.reference_splines.size() == 0) {
    return;
  }
  this->drawRoadSideFromWidth(painter);
  this->drawSweeper(painter);
  this->drawTrackTargetWithPoints(painter);
  this->drawNewTarget(painter);
  this->drawPlanningPoint(painter);
  this->drawPlanningCandidatesSplines(painter);
  this->drawPlanningSplines(painter);
  this->drawGarbageResults(painter);
  this->drawText(painter);
}

/*******************************************************
 * @brief 绘制扫地车
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawSweeper(QPainter &painter)
{
  painter.save();
  QPen pen;

  // front of vehicle
  const double VEH_W = m_planningData.front_vehicle_width;
  const double VEH_L = m_planningData.front_vehicle_length;
  const double VEH_HEAD = m_planningData.head_point.x;

  QRectF rectfSweeper = QRectF(-VEH_L + VEH_HEAD, -VEH_W / 2, VEH_L, VEH_W);
  QPolygonF pgfSweeper = m_transform.map(rectfSweeper);
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawPolygon(pgfSweeper);

  // front line
  QLineF linef(m_planningData.front_axle_center.x, m_planningData.front_axle_center.y,
               m_planningData.hinge_point.x, m_planningData.hinge_point.y);
  linef = m_transform.map(linef);
  pen.setColor(Qt::green);
  pen.setWidth(2);
  painter.setPen(pen);
  painter.drawLine(linef);

  // back line
  linef = QLineF(m_planningData.hinge_point.x, m_planningData.hinge_point.y,
                 m_planningData.rear_axle_center.x, m_planningData.rear_axle_center.y
               );
  linef = m_transform.map(linef);
//  pen.setColor(Qt::green);
//  painter.setPen(pen);
  painter.drawLine(linef);

  // point
  painter.setBrush(Qt::green);
  QPointF ptf = linef.p1();
  painter.drawEllipse(ptf, 2, 2);

  // back vehicle
  linef = QLineF(m_planningData.hinge_point.x, m_planningData.hinge_point.y,
                 m_planningData.rear_axle_center.x, m_planningData.rear_axle_center.y
               );

  QPolygonF pgf;
  ptf = QPointF(m_planningData.rear_point.x, m_planningData.rear_point.y);
  QLineF linef2;
  linef2.setP1(ptf);
  linef2.setLength(m_planningData.rear_vehicle_width / 2.0);
  linef2.setAngle(linef.normalVector().angle());
  pgf << linef2.p2();
  QPointF ptfStart = linef2.p2();
  linef2.setAngle(linef.normalVector().angle() + 180);
  pgf << linef2.p2();

  linef2 = QLineF(m_planningData.rear_point.x, m_planningData.rear_point.y,
                 m_planningData.hinge_point.x, m_planningData.hinge_point.y
               );
  linef2.setLength(m_planningData.rear_vehicle_length);
  ptf = linef2.p2();

  linef2.setP2(QPointF(1000000.0, 0));
  linef2.setP1(ptf);
  linef2.setLength(m_planningData.rear_vehicle_width / 2.0);
  linef2.setAngle(linef.normalVector().angle() + 180);
  pgf << linef2.p2();
  linef2.setAngle(linef.normalVector().angle());
  pgf << linef2.p2();
  pgf << ptfStart;

  pgf = m_transform.map(pgf);
  painter.drawPolyline(pgf);

  // stop line
  /*const double STOP_DIS = 0.5;
  QPolygonF pgf;
  pgf << QPointF(-VEH_L + VEH_HEAD, VEH_W / 2 + STOP_DIS) <<
         QPointF(VEH_HEAD + STOP_DIS, VEH_W / 2 + STOP_DIS) <<
         QPointF(VEH_HEAD + STOP_DIS, -VEH_W / 2 - STOP_DIS) <<
         QPointF(-VEH_L + VEH_HEAD, -VEH_W / 2 - STOP_DIS);
  pgf = m_transform.map(pgf);
  pen.setStyle(Qt::DotLine);
  painter.setPen(pen);
  painter.drawPolygon(pgf);

  // pass line
  const double PASS_DIS = 1.1;
  pgf.clear();
  pgf << QPointF(-VEH_L + VEH_HEAD, VEH_W / 2 + PASS_DIS) <<
         QPointF(VEH_HEAD + PASS_DIS, VEH_W / 2 + PASS_DIS) <<
         QPointF(VEH_HEAD + PASS_DIS, -VEH_W / 2 - PASS_DIS) <<
         QPointF(-VEH_L + VEH_HEAD, -VEH_W / 2 - PASS_DIS);
  pgf = m_transform.map(pgf);
  pen.setColor(Qt::darkMagenta);
  painter.setPen(pen);
  painter.drawPolygon(pgf);*/

  painter.restore();

  this->drawRadar(painter);
  this->drawUltrasonic(painter);
}

void QPlanningShowWidget::drawUltrasonic(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setStyle(Qt::SolidLine);
  painter.setBrush(Qt::darkRed);
  painter.setFont(QFont("Times", 10));

  const auto points = m_planningData.ultrasonic_points;
  QPointF ptf;

  for (int i = 0; i < 14; ++i) {
    if (qAbs<double>(points[i].x) < 0.001) {
      ptf = QPointF(m_planningData.head_point.x,
                    m_planningData.front_vehicle_width * 0.3 * qPow(-1, i));
    }
    else {
      ptf = QPointF(points[i].x, points[i].y);
    }
    QPainterPath path;
    path.addEllipse(ptf, 0.08, 0.08);
    path = m_transform.map(path);

    pen.setColor(Qt::darkRed);
    painter.setPen(pen);
    painter.drawPath(path);

    pen.setColor(Qt::white);
    painter.setPen(pen);
    painter.drawText(path.boundingRect(), Qt::AlignCenter, QString::number(i));
  }

  painter.restore();
}

void QPlanningShowWidget::drawRadar(QPainter &painter)
{
  painter.save();

  const auto radar_point = m_planningData.radar_Point;
  QPointF ptf;
  if (qAbs<double>(radar_point.x) < 0.001) {
    ptf = QPointF(m_planningData.head_point.x, m_planningData.head_point.y);
  }
  else {
    ptf = QPointF(radar_point.x, radar_point.y);
  }
  QRectF rectf(0, 0, 0.06, 0.12);
  rectf.moveCenter(ptf);

  QPen pen;
  pen.setStyle(Qt::SolidLine);
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.setBrush(Qt::darkBlue);

  QPolygonF pgf = m_transform.map(rectf);
  QPainterPath path;
  path.addPolygon(pgf);
  painter.drawPath(path);

  painter.restore();
}

/**
 * @brief 绘制参考线，路边沿(根据参考线左侧、右侧路宽数据绘制）
 * @param painter
 */
void QPlanningShowWidget::drawRoadSideFromWidth(QPainter &painter)
{
  painter.save();

  // reference
  QPen pen;
  pen.setWidth(1);
  pen.setColor(Qt::black);
  pen.setStyle(Qt::DashLine);
  painter.setPen(pen);

  for (const auto &spline : m_planningData.reference_splines) {
    QPainterPath path(QPointF(spline.xb.x, spline.yb.x));
    path.cubicTo(
          spline.xb.y, spline.yb.y,
          spline.xb.z, spline.yb.z,
          spline.xb.w, spline.yb.w
          );
    painter.drawPath(m_transform.map(path));
  }

  // reference points size
  const auto &points = m_planningData.reference_points;
  const int size_ref_points = points.size();
  if (size_ref_points == 0) {
    return;
  }

  // road side points
  pen.setWidth(4);
  pen.setColor(Qt::yellow);
  pen.setStyle(Qt::SolidLine);
  painter.setPen(pen);
  QVector<QPointF> points_left, points_right;
  for (int i = 0; i < size_ref_points; ++i) {
    QPointF ptf_left, ptf_right;
    double l = points[i].l + points[i].left_road_width;
    if (points[i].left_road_width < 0.01) {
      l = points[i].l + 6.6;
    }
    this->slToXy(points[i].s, l, ptf_left);
    int index = this->findReferenceIndex(ptf_left.x(), ptf_left.y());
    if (index != i) {
      QPointF ptf;
      this->slToXy(points[index].s, l, ptf);
      ptf_left = (ptf_left + ptf) / 2;
    }
    points_left << ptf_left;

    l = points[i].l - points[i].right_road_width;
    if (points[i].right_road_width < 0.01) {
      l = points[i].l - 0.875;
    }
    this->slToXy(points[i].s, l, ptf_right);
    index = this->findReferenceIndex(ptf_right.x(), ptf_right.y());
    if (index != i) {
      QPointF ptf;
      this->slToXy(points[index].s, l, ptf);
      ptf_right = (ptf_right + ptf) / 2;
    }
    points_right << ptf_right;
  }

  // draw left bezier
  QBezierCurve bezierCurve;
  for (const auto &ptf : points_left) {
    bezierCurve.addPoint(ptf);
  }
  bezierCurve.calcBezier();
  const auto &beziers_left = bezierCurve.getBezier();
  for (const auto &bezier : beziers_left) {
    QPainterPath path(bezier.start);
    path.cubicTo(bezier.control, bezier.control2, bezier.end);
//    painter.drawPath(m_transform.map(path));
  }

  // draw right bezier
  bezierCurve.clear();
  for (const auto &ptf : points_right) {
    bezierCurve.addPoint(ptf);
  }
  bezierCurve.calcBezier();
  const auto &beziers_right = bezierCurve.getBezier();
  for (const auto &bezier : beziers_right) {
    QPainterPath path(bezier.start);
    path.cubicTo(bezier.control, bezier.control2, bezier.end);
//    painter.drawPath(m_transform.map(path));
  }

  // draw point
  QPolygonF pgf;
  for (const auto &ptf : points_left) {
    pgf << ptf;
  }
  painter.drawPolyline(m_transform.map(pgf));

  pgf.clear();
  for (const auto &ptf : points_right) {
    pgf << ptf;
  }
  painter.drawPolyline(m_transform.map(pgf));

  painter.restore();
}

/*******************************************************
 * @brief 绘制决策点
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawDecisionPoint(QPainter &painter)
{
  const auto &pts = m_planningData.reference_points;

  const int SIZE = m_planningData.reference_points.size();
  int index = -1;
  double dS = 0;//m_planningData.decision_point_s;
  double dL = 0;//m_planningData.decision_point_l;
  for (int i = 0; i < SIZE - 1; ++i) {
    if (dS >= pts[i].s && dS < pts[i + 1].s) {
      index = i;
      break;
    }
  }
  QPointF ptfDecision;
  if (index == -1) {
    if (dS < pts[0].s) {
      ptfDecision = QPointF(pts[0].x - (pts[0].s - dS), pts[0].y + dL);
    }
    else {
      ptfDecision = QPointF(pts[SIZE - 1].x + (pts[SIZE - 1].s - dS), pts[0].y + dL);
    }
  }
  else {
    QLineF linef(pts[index].x, pts[index].y, pts[index + 1].x, pts[index + 1].y);
    double factor = (dS - pts[index].s) / (pts[index + 1].s - pts[index].s);
    ptfDecision = linef.pointAt(factor);
    QLineF linefNormal;
    if ((dS - pts[index].s) > (pts[index + 1].s - dS)) {
      linef = QLineF(ptfDecision, QPointF(pts[index].x, pts[index].y));
      linefNormal = linef.normalVector();
    }
    else {
      linef = QLineF(ptfDecision, QPointF(pts[index + 1].x, pts[index + 1].y));
      linefNormal = linef.normalVector();
      linefNormal.setAngle(linefNormal.angle() + 180);
    }
    linefNormal.setLength(dL);
    ptfDecision = linefNormal.p2();
  }

  QPointF ptf = m_transform.map(ptfDecision);
  QRect rect = QRect(0, 0, 13, 13);
  rect.moveCenter(ptf.toPoint());
  QLine line(rect.topLeft(), rect.bottomRight());
  QLine line2(rect.bottomLeft(), rect.topRight());

  painter.save();
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(2);
  painter.setPen(pen);
  painter.drawLine(line);
  painter.drawLine(line2);
  painter.restore();
}

/*******************************************************
 * @brief 绘制规划点
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawPlanningPoint(QPainter &painter)
{
  QPointF ptfPlanning(-1000.0, -1000.0);
  if (m_nCostType == OLD_COST) {
    ptfPlanning = QPointF(
          m_planningData.planning_output.pose.position.x,
          m_planningData.planning_output.pose.position.y
          );
  }
  else {
    double arch_len = 0.0;
    QPointF ptfStart, ptfEnd, ptfControl1, ptfControl2;
    for (const auto &spline : m_planningData.planning_output.trajectory.splines) {
      ptfStart = QPointF(spline.xb.x, spline.yb.x);
      ptfEnd = QPointF(spline.xb.w, spline.yb.w);
      ptfControl1 = QPointF(spline.xb.y, spline.yb.y);
      ptfControl2 = QPointF(spline.xb.z, spline.yb.z);

      QPainterPath path(ptfStart);
      path.cubicTo(ptfControl1, ptfControl2, ptfEnd);

      if ( (arch_len + path.length()) >= 0.45 ) {
        arch_len = 0.45 - arch_len;
        qreal percent = path.percentAtLength(arch_len);
        ptfPlanning = path.pointAtPercent(percent);
        break;
      }
      arch_len += path.length();
    }
  }
  if (ptfPlanning.x() > -999.0) {
    QPointF ptf = m_transform.map(ptfPlanning);
    QRect rect = QRect(0, 0, 13, 13);
    rect.moveCenter(ptf.toPoint());
    QLineF line(rect.topLeft(), rect.bottomRight());
    QLineF line2(rect.bottomLeft(), rect.topRight());

    painter.save();
    QPen pen;
    pen.setColor(Qt::darkGreen);
    pen.setWidth(4);
    painter.setPen(pen);
    painter.drawLine(line);
    painter.drawLine(line2);
    painter.restore();
  }
}

/*******************************************************
 * @brief 绘制planning path of bezier
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawPlanningSplines(QPainter &painter)
{
  QPen pen;
  pen.setWidth(2);
  pen.setBrush(Qt::blue);
  painter.save();
  painter.setPen(pen);

  const auto &val_planning_splines = m_planningData.planning_output.trajectory.splines;
  this->drawBezierLine(painter, val_planning_splines);
  painter.restore();

  const int size_splines = val_planning_splines.size();
  if (size_splines > 0) {
    pen.setWidth(1);
    pen.setColor(Qt::black);
    painter.setFont(QFont("Times", 12));
    painter.save();
    painter.setPen(pen);

    QPointF ptf(val_planning_splines[size_splines - 1].xb.w,
        val_planning_splines[size_splines - 1].yb.w);
    painter.drawText(m_transform.map(ptf), QString::number(m_planningData.planning_output.trajectory.id));

    painter.restore();
  }
}

void QPlanningShowWidget::drawPlanningPoints(QPainter &painter)
{
  /*QPolygonF pgf;
  for (const auto &point : m_planningData.planning_trajectory.points) {
    pgf << QPointF(point.x, point.y);
  }
  if (pgf.size() > 0) {
    pgf = m_transform.map(pgf);

    painter.save();

    QPen pen;
    pen.setWidth(2);
    pen.setBrush(Qt::blue);
    painter.setPen(pen);
    painter.drawPolyline(pgf);

    pen.setWidth(1);
    pen.setColor(Qt::black);
    painter.setFont(QFont("Times", 12));
    painter.setPen(pen);

    const auto &points = m_planningData.planning_trajectory.points;
    QPointF ptf(points[points.size() - 1].x, points[points.size() - 1].y);
    painter.drawText(m_transform.map(ptf),
                     QString::number(m_planningData.planning_trajectory.id));

    painter.restore();
  }*/
}

/*******************************************************
 * @brief 绘制candidates planning path of bezier
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawPlanningCandidatesSplines(QPainter &painter)
{
  QPen pen;
  pen.setWidth(1);
  pen.setColor(Qt::magenta);
  painter.setFont(QFont("Times", 12));
  painter.setPen(pen);

  auto candidates = m_planningData.planning_trajectory_candidates;
  int size_candidates = candidates.size();
  double value[QPlanningCostWidget::Count];
  QCostValueWidget::getCostValue(value);
  if (m_nCostType == NEW_COST) {
    for (int i = 0; i < size_candidates; ++ i) {
      candidates[i].cost = value[QPlanningCostWidget::Safety] * candidates[i].safety_cost
          + value[QPlanningCostWidget::Lateral] * candidates[i].lateral_cost
          + value[QPlanningCostWidget::Smoothness] * candidates[i].smoothness_cost
          + value[QPlanningCostWidget::Consistency] * candidates[i].consistency_cost
          + value[QPlanningCostWidget::Garbage] * candidates[i].garbage_cost;
    }
  }

  int planning_id = m_planningData.planning_output.trajectory.id;
  if (size_candidates > 10) {
    using type_candidates = decltype(candidates[0]);
    std::sort(candidates.begin(), candidates.end(), [](const type_candidates &val,
              const type_candidates &val2){
      return val.cost < val2.cost;
    });
  }
  int decision = static_cast<int>(m_planningData.overall_decision);
  size_candidates = qBound<int>(0, size_candidates, 10);
  if (decision >= 0 && decision < 4) {
    for (int i = 0; i < size_candidates; ++ i) {
      int candidate_id = candidates[i].id;
      if (planning_id == candidate_id) {
        continue;
      }

      const auto &val_candidate_splines = candidates[i].splines;
      const int SIZE = static_cast<int>(val_candidate_splines.size());
      this->drawBezierLine(painter, val_candidate_splines);

      if (SIZE > 2) {
        int offset = (i % 2) == 0 ? 2 : 3;
        QPointF ptf(val_candidate_splines[SIZE - offset].xb.w, val_candidate_splines[SIZE - offset].yb.w);
        painter.drawText(m_transform.map(ptf), QString::number(candidate_id));
      }
    }
  }
}


/*******************************************************
 * @brief 绘制垃圾检测结果
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawGarbageResults(QPainter &painter)
{
  const auto &garbage_results = m_planningData.garbage_detection_results;
  const int size = static_cast<int>(garbage_results.size());

  painter.save();
  QPen pen;
  pen.setWidth(2);
  pen.setColor(Qt::darkGreen);
  painter.setFont(QFont("Times", 10));

  for (int i = 0; i < size; ++i) {
    if (i < size - m_nNewGarbageCount) {
      pen.setStyle(Qt::SolidLine);
    }
    else {
      pen.setStyle(Qt::DotLine);
    }
    painter.setPen(pen);

    QLineF linef(0, 0, 2, 0);
    linef.setAngle(-garbage_results[i].angle * 180.0 / PI);
    linef.setLength(garbage_results[i].distance);
    QRectF rectf(0, 0, garbage_results[i].width, garbage_results[i].length);
    rectf.moveCenter(linef.p2());

    QPolygonF pgf = m_transform.map(rectf);
    painter.drawPolygon(pgf);

//    QString text = QString("D: %1 A: %2").
//        arg(garbage_results[i].distance, 5, 'f', 2).
//        arg(garbage_results[i].angle, 5, 'f', 3);
//    painter.drawText(m_transform.map(rectf.topRight()), text);
  }

  painter.restore();
}

void QPlanningShowWidget::drawUltrasonicTarget(QPainter &painter)
{
  painter.save();

  const auto &ultrasonic = m_planningData.ultrasonic_results;

  const double VEH_W = m_planningData.front_vehicle_width;
  const double VEH_L = m_planningData.front_vehicle_length;
  const double VEH_HEAD = m_planningData.head_point.x;

  const double STOP_DIS = 0.5;
  const double PASS_DIS = 1.1;
  const int SIZE = static_cast<int>(ultrasonic.size());
  for (int i = 0; i < SIZE; ++i) {
    const int ID = static_cast<int>(ultrasonic[i].radar_pos_id);
    const double DIS = ultrasonic[i].distance;
    if (DIS < 0.001 || DIS > PASS_DIS) continue;
    if (ID < 8 || ID > 15) continue;

    QPointF ptf;
    switch (ID) {
      case 8:
        ptf = QPointF(-VEH_L + VEH_HEAD + 0.13, VEH_W / 2 + DIS);
        break;
      case 9:
        ptf = QPointF(-VEH_L + VEH_HEAD + 1.1, VEH_W / 2 + DIS);
        break;
      case 10:
        ptf = QPointF(-VEH_L + VEH_HEAD + 2.08, VEH_W / 2 + DIS);
        break;
      case 11:
        ptf = QPointF(VEH_HEAD + 0.05 + DIS / 1.1412, VEH_W / 2 + 0.05 + DIS / 1.1412);
        break;
      case 12:
        ptf = QPointF(-VEH_L + VEH_HEAD + 0.13, -VEH_W / 2 - DIS);
        break;
      case 13:
        ptf = QPointF(-VEH_L + VEH_HEAD + 1.1, -VEH_W / 2 - DIS);
        break;
      case 14:
        ptf = QPointF(-VEH_L + VEH_HEAD + 2.08, -VEH_W / 2 - DIS);
        break;
      case 15:
        ptf = QPointF(VEH_HEAD + 0.05 + DIS / 1.1412, -VEH_W / 2 - 0.05 - DIS / 1.1412);
        break;
      default:
        break;
    }

    if (DIS < STOP_DIS) {
      painter.setPen(Qt::magenta);
    }
    else {
      painter.setPen(Qt::blue);
    }
    QRect rect(0, 0, 12, 12);
    rect.moveCenter(m_transform.map(ptf).toPoint());
    painter.drawRect(rect);
  }

  painter.restore();
}

/*******************************************************
 * @brief 绘制雷达障碍物，路边沿以内
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawRadarTarget(QPainter &painter)
{
  painter.save();

  const auto &radar = m_planningData.radar_results;

  const double VEH_W = m_planningData.front_vehicle_width;
  const double VEH_L = m_planningData.front_vehicle_length;
  const double VEH_HEAD = m_planningData.head_point.x;

  const double STOP_DIS = 0.5;
  const double PASS_DIS = 1.1;
  const int SIZE = static_cast<int>(radar.size());
  for (int i = 0; i < SIZE; ++i) {
    const int ID = static_cast<int>(radar[i].devid);
    const double DIS_X = radar[i].range_lat;
    const double DIS_Y = radar[i].range_lon;
    if (ID < 10 || ID > 13) continue;

    QPointF ptf;
    switch (ID) {
      case 10:
        ptf = QPointF(VEH_HEAD - 0.7 - DIS_X, VEH_W / 2 + DIS_Y);
        break;
      case 11:
        ptf = QPointF(VEH_HEAD - 0.7 - DIS_X, -VEH_W / 2 - DIS_Y);
        break;
      case 12:
        ptf = QPointF(-VEH_L + VEH_HEAD + 0.7 - DIS_X, VEH_W / 2 - DIS_Y);
        break;
      case 13:
        ptf = QPointF(-VEH_L + VEH_HEAD + 0.7 - DIS_X, VEH_W / 2 + DIS_Y);
        break;
    }

    if ( ptf.x() < (-VEH_L + VEH_HEAD) || ptf.x() > VEH_HEAD + PASS_DIS ||
         ptf.y() < (-VEH_W / 2 - PASS_DIS)  || ptf.y() > (VEH_W / 2 + PASS_DIS) ) {
      continue;
    }

    if ( ptf.x() >= (-VEH_L + VEH_HEAD) && ptf.x() <= VEH_HEAD + STOP_DIS &&
         ptf.y() >= (-VEH_W / 2 - STOP_DIS) && ptf.y() <= (VEH_W / 2 + STOP_DIS) ) {
      painter.setPen(Qt::magenta);
    }
    else {
      painter.setPen(Qt::blue);
    }
    QRect rect(0, 0, 12, 12);
    rect.moveCenter(m_transform.map(ptf).toPoint());
    painter.drawRect(rect);
  }

  painter.restore();
}

/*******************************************************
 * @brief 绘制激光障碍物，路边沿以内
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawTrackTarget(QPainter &painter)
{
  const auto &TRACKS = m_planningData.fusion_results;
  const int SIZE = static_cast<int>(m_planningData.fusion_results.size());
  const int SIZE_REF = static_cast<int>(m_planningData.reference_points.size());

  painter.save();
  QPen pen;
  pen.setColor(Qt::darkMagenta);
  pen.setStyle(Qt::SolidLine);
  painter.setFont(QFont("Times", 10));
  painter.setPen(pen);

  for (int i = 0; i < SIZE; ++i) {
    QPolygonF pgf = this->createSlPgf(
          QPointF(TRACKS[i].X, TRACKS[i].Y), TRACKS[i].W, TRACKS[i].L);
    if (pgf.empty()) {
      continue;
    }

    bool contains = false;
    foreach (const QPointF &ptf, pgf) {
      int indexLeft = -1;
      int indexRight = -1;

      // find x range
      const qreal x_max = 10.0 + m_planningData.head_point.x;
      for (int j = 0; j < SIZE_REF - 1; ++j) {
        if (indexLeft != -1 && indexRight != -1) {
          break;
        }
        if (m_ptfsLeftRoadSide[j].x() > x_max || m_ptfsRightRoadSide[j].x() > x_max) {
          break;
        }
        if (indexLeft == -1 && ptf.x() > m_ptfsLeftRoadSide[j].x() &&
             ptf.x() <= m_ptfsLeftRoadSide[j + 1].x()) {
          indexLeft = j;
        }
        if (indexRight == -1 && ptf.x() > m_ptfsRightRoadSide[j].x() &&
            ptf.x() <= m_ptfsRightRoadSide[j + 1].x()) {
          indexRight = j;
        }
      }
      if (indexLeft == -1 && indexRight == -1) {
        continue;
      }
      // check y range
      contains = ( (indexLeft != -1 && ptf.y() <= m_ptfsLeftRoadSide[indexLeft].y()) &&
          (indexRight != -1 && ptf.y() >= m_ptfsRightRoadSide[indexRight].y()) );
      if (contains) {
        break;
      }
    }
    if (contains) {
      pgf = m_transform.map(pgf);
      painter.drawPolygon(pgf);
      painter.drawText(pgf.boundingRect(), Qt::AlignCenter, QString::number(i));
    }
  }

  painter.restore();
}

/*******************************************************
 * @brief 绘制激光障碍物，路边沿以内，以角点画图
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawTrackTargetWithPoints(QPainter &painter)
{
  const auto &TRACKS = m_planningData.fusion_results;
  const int SIZE = static_cast<int>(m_planningData.fusion_results.size());
  const int SIZE_REF = static_cast<int>(m_planningData.reference_points.size());

  painter.save();
  QPen pen;
  pen.setColor(Qt::darkMagenta);
  pen.setStyle(Qt::SolidLine);
  painter.setFont(QFont("Times", 10));
  painter.setPen(pen);

  for (int i = 0; i < SIZE; ++i) {
    QPolygonF pgf;
    for (const auto &point : TRACKS[i].edge_points) {
      pgf << QPointF(point.x, point.y);
    }

    bool contains = m_bFlagShowAllTargets;
    const qreal x_max = 10.0 + m_planningData.head_point.x;
    if (!m_bFlagShowAllTargets) {
      foreach (const QPointF &ptf, pgf) {
        int indexLeft = -1;
        int indexRight = -1;

        // find x range
        for (int j = 0; j < SIZE_REF - 1; ++j) {
          if (indexLeft != -1 && indexRight != -1) {
            break;
          }
          if (m_ptfsLeftRoadSide[j].x() > x_max || m_ptfsRightRoadSide[j].x() > x_max) {
            break;
          }
          if (indexLeft == -1 && ptf.x() > m_ptfsLeftRoadSide[j].x() &&
              ptf.x() <= m_ptfsLeftRoadSide[j + 1].x()) {
            indexLeft = j;
          }
          if (indexRight == -1 && ptf.x() > m_ptfsRightRoadSide[j].x() &&
              ptf.x() <= m_ptfsRightRoadSide[j + 1].x()) {
            indexRight = j;
          }
        }
        if (indexLeft == -1 && indexRight == -1) {
          continue;
        }
        // check y range
        contains = ( (indexLeft != -1 && ptf.y() <= m_ptfsLeftRoadSide[indexLeft].y()) &&
                     (indexRight != -1 && ptf.y() >= m_ptfsRightRoadSide[indexRight].y()) );
        if (contains) {
          break;
        }
      }
    }
    if (contains) {
      pgf = m_transform.map(pgf);
      painter.drawPolygon(pgf);
      painter.drawText(pgf.boundingRect(), Qt::AlignCenter, QString::number(i));
    }
  }

  // new targets
  pen.setStyle(Qt::DotLine);
  painter.setPen(pen);
  for (int i = SIZE - m_nNewTracksCount; i < SIZE; ++i) {
    QPolygonF pgf;
    for (const auto &point : TRACKS[i].edge_points) {
      pgf << QPointF(point.x, point.y);
    }
    pgf = m_transform.map(pgf);
    painter.drawPolygon(pgf);
    painter.drawText(pgf.boundingRect(), Qt::AlignCenter, QString::number(i));
  }

  painter.restore();
}

/*******************************************************
 * @brief 绘制贝塞尔曲线
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawBezierLine(
    QPainter &painter,
    const std::vector< ::debug_tool::ads_Spline_<std::allocator<void>>> &splines)
{
  QPointF ptfStart, ptfEnd, ptfControl1, ptfControl2;
  for (const auto &spline : splines) {
    ptfStart = QPointF(spline.xb.x, spline.yb.x);
    ptfEnd = QPointF(spline.xb.w, spline.yb.w);
    ptfControl1 = QPointF(spline.xb.y, spline.yb.y);
    ptfControl2 = QPointF(spline.xb.z, spline.yb.z);
    ptfStart = m_transform.map(ptfStart);
    ptfEnd = m_transform.map(ptfEnd);
    ptfControl1 = m_transform.map(ptfControl1);
    ptfControl2 = m_transform.map(ptfControl2);

    QPainterPath path(ptfStart);
    path.cubicTo(ptfControl1, ptfControl2, ptfEnd);
    painter.drawPath(path);
  }
}

void QPlanningShowWidget::drawNewTarget(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setColor(Qt::darkMagenta);
  pen.setStyle(Qt::DotLine);
  painter.setPen(pen);

  const auto size = m_ptfTargets.size();
  if (size == 1) {
    QLineF linef(m_ptfTargets[0], m_ptfTargetMove);
    painter.drawLine(linef);
  }
  else if (size == 2) {
    QPolygonF pgf = this->createTargetPgf(m_ptfTargets, m_ptfTargetMove);
    painter.drawPolyline(pgf);
  }

  painter.restore();
}

void QPlanningShowWidget::drawText(QPainter &)
{

}

/**
 * @brief 根据s坐标查找位于参考点的区域范围
 * @param s: s坐标系

 * @return -1: 起点之外, SIZE - 1: 终点之外, 其它: index, index+1之间
*/
int QPlanningShowWidget::findReferenceIndex(const double s)
{
  const auto &REF_PTS = m_planningData.reference_points;
  const int SIZE = m_planningData.reference_points.size();
  int index = -1;

  // find range of s
  if (s < REF_PTS[0].s) {
    index = -1;
  }
  else if (s >= REF_PTS[SIZE - 1].s) {
    index = SIZE - 1;
  }
  else {
    for (int i = 0; i < SIZE - 1; ++i) {
      if (s >= REF_PTS[i].s && s < REF_PTS[i + 1].s) {
        index = i;
        break;
      }
    }
  }

  return index;
}

/**
 * @brief 查找最近的参考线点
 * @param x
 * @param y
 * @return 参考点序号
 */
int QPlanningShowWidget::findReferenceIndex(const double x, const double y)
{
  int index = -1;
  double distance = 100000;

  const auto &points = m_planningData.reference_points;
  const int size_pts = points.size();
  for (int i = 0; i < size_pts; ++i) {
    QLineF linef(x, y, points[i].x, points[i].y);
    if (linef.length() < distance) {
      distance = linef.length();
      index = i;
    }
  }

  return index;
}

/**
 * @brief xy坐标转换为sl坐标
 * @param ptfXy: 车体坐标系，x, 车头正向, y, 车左侧
 * @param s: sl坐标系, s, 参考线切线方向
 * @param s: sl坐标系, l, 参考线垂直向左

 * @return
*/
void QPlanningShowWidget::xyToSl(const QPointF &ptfXy, double &s, double &l)
{
  const auto &REF_PTS = m_planningData.reference_points;
  const int SIZE = m_planningData.reference_points.size();
  int index = -1;
  for (int i = 0; i < SIZE - 1; ++i) {
    QLineF linef(REF_PTS[i].x, REF_PTS[i].y, REF_PTS[i + 1].x, REF_PTS[i + 1].y);
    QLineF linef2(REF_PTS[i].x, REF_PTS[i].y, ptfXy.x(), ptfXy.y());
    double angle = linef.angleTo(linef2);
    if (angle > 90 && angle < 270) {
      continue;
    }

    linef = QLineF(REF_PTS[i + 1].x, REF_PTS[i + 1].y, REF_PTS[i].x, REF_PTS[i].y);
    linef2 = QLineF(REF_PTS[i + 1].x, REF_PTS[i + 1].y, ptfXy.x(), ptfXy.y());
    angle = linef.angleTo(linef2);
    if (angle <= 90 || angle >= 270) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    l = 1.0e8;
    for (int i = 0; i < SIZE; ++i) {
      double len = QLineF(REF_PTS[i].x, REF_PTS[i].y, ptfXy.x(), ptfXy.y()).length();
      if (l > len) {
        index = i;
        l = len;
      }
    }
    s = REF_PTS[index].s;
    if (index > 1 && index < SIZE - 1) {
      double angle = QLineF(ptfXy, QPointF(REF_PTS[index - 1].x, REF_PTS[index - 1].y)).angleTo(
            QLineF(ptfXy, QPointF(REF_PTS[index + 1].x, REF_PTS[index + 1].y)));
      if (angle < 180) {
        l *= -1;
      }
    }
    return;
  }

  QLineF linef(REF_PTS[index].x, REF_PTS[index].y, REF_PTS[index + 1].x, REF_PTS[index + 1].y);
  QLineF linef2(ptfXy.x(), ptfXy.y(), ptfXy.x(), ptfXy.y() + 1);
  linef2.setAngle(linef.normalVector().angle());

  QPointF ptfInter;
  QLineF::IntersectType type = linef.intersect(linef2, &ptfInter);
  if (type == QLineF::NoIntersection) {
    return;
  }

  l = QLineF(ptfXy, ptfInter).length();
  double angle = QLineF(ptfXy, QPointF(REF_PTS[index].x, REF_PTS[index].y)).angleTo(
        QLineF(ptfXy, QPointF(REF_PTS[index + 1].x, REF_PTS[index + 1].y)));
  if (angle < 180) {
    l *= -1;
  }
  s = QLineF(REF_PTS[index].x, REF_PTS[index].y, ptfInter.x(), ptfInter.y()).length();
  s += REF_PTS[index].s;

  return;
}

/**
 * @brief sl坐标转换为xy坐标
 * @param s: sl坐标系, s, 参考线切线方向
 * @param s: sl坐标系, l, 参考线垂直向左
 * @param ptfXy: 车体坐标系，x, 车头正向, y, 车左侧

 * @return
*/
void QPlanningShowWidget::slToXy(const double s, const double l, QPointF &ptfXy)
{
  const auto &REF_PTS = m_planningData.reference_points;
  const int SIZE = m_planningData.reference_points.size();

  const int index = this->findReferenceIndex(s);
  bool direct = true;  // line direction, ture, forward
  QPointF ptfStart;
  QPointF ptfEnd;
  if (index == -1) {
    direct = true;
    QLineF linef(REF_PTS[1].x, REF_PTS[1].y, REF_PTS[0].x, REF_PTS[0].y);
    linef.setLength(REF_PTS[1].s - s);
    ptfStart = linef.p2();
    ptfEnd = linef.p1();
  }
  else if (index == SIZE - 1) {
    direct = false;
    QLineF linef(REF_PTS[SIZE - 2].x, REF_PTS[SIZE - 2].y,
        REF_PTS[SIZE - 1].x, REF_PTS[SIZE - 1].y);
    linef.setLength(s - REF_PTS[SIZE - 2].s);
    ptfStart = linef.p2();
    ptfEnd = linef.p1();
  }
  else {
    QLineF linef(REF_PTS[index].x, REF_PTS[index].y,
                 REF_PTS[index + 1].x, REF_PTS[index + 1].y);
    ptfStart = linef.pointAt(
          (s - REF_PTS[index].s) / (REF_PTS[index + 1].s - REF_PTS[index].s) );
    if ( (s - REF_PTS[index].s) > (REF_PTS[index + 1].s - s) ) {
      direct = false;
      ptfEnd = QPointF(REF_PTS[index].x, REF_PTS[index].y);
    }
    else {
      direct = true;
      ptfEnd = QPointF(REF_PTS[index + 1].x, REF_PTS[index + 1].y);
    }
  }

  //
  QLineF linef(ptfStart, ptfEnd);
  linef = linef.normalVector();
  if (direct) {
    linef.setAngle(linef.angle() + 180);
  }
  linef.setLength(l);
  ptfXy = linef.p2();
}

/**
 * @brief sl坐标元素转换为xy坐标系QPolygonF
 * @param s: center, 中心坐标
 * @param width: s向长度
 * @param length: l向长度
 * @param slflag: true, 中心坐标为sl坐标系, 中心坐标为xy坐标系

 * @return
*/
QPolygonF QPlanningShowWidget::createSlPgf(const QPointF &center, double width, double length,
                                           bool slflag)
{
  const auto &REF_PTS = m_planningData.reference_points;
  const int SIZE = m_planningData.reference_points.size();

  QPolygonF pgf;
  double s = 0, l = 0;
  if (slflag) {
    s = center.x();
    l = center.y();
  }
  else {
    this->xyToSl(center, s, l);
  }
  const int index = this->findReferenceIndex(s);

  QLineF linef;
  QPointF ptf, ptf2;
  this->slToXy(s, l - width / 2, ptf);
  this->slToXy(s, l + width / 2, ptf2);

  if (index == -1) {
    linef = QLineF(REF_PTS[0].x, REF_PTS[0].y, REF_PTS[1].x, REF_PTS[1].y);
  }
  else if (index == SIZE - 1) {
    linef = QLineF(REF_PTS[SIZE - 2].x, REF_PTS[SIZE - 2].y,
        REF_PTS[SIZE - 1].x, REF_PTS[SIZE - 1].y);
  }
  else {
    linef = QLineF(REF_PTS[index].x, REF_PTS[index].y,
                   REF_PTS[index + 1].x, REF_PTS[index + 1].y);
  }

  QLineF linef2 = QLineF(ptf.x(), ptf.y(), ptf.x() + 2, ptf.y());
  linef2.setAngle(linef.angle());
  linef2.setLength(length / 2);
  pgf << linef2.p2();
  linef2.setAngle(linef.angle() + 180);
  pgf << linef2.p2();

  linef2 = QLineF(ptf2.x(), ptf2.y(), ptf2.x() + 2, ptf2.y());
  linef2.setAngle(linef.angle() + 180);
  linef2.setLength(length / 2);
  pgf << linef2.p2();
  linef2.setAngle(linef.angle());
  pgf << linef2.p2();

  return pgf;
}

void QPlanningShowWidget::addTracksToData()
{
  auto &tracks = m_planningData.fusion_results;
  const int size = static_cast<int>(m_planningData.fusion_results.size());

  QPolygonF pgf = this->createTargetPgf(m_ptfTargets, m_ptfTargetMove);
  QPolygon pg = pgf.toPolygon();

  int trackId = size == 0 ? 0 : tracks[size - 1].TRACK_ID;
  debug_tool::ads_TrackTarget_<std::allocator<void>> track;
  track.TRACK_ID = trackId + 1;

  QPointF ptf(pg.point(0).x(), pg.point(0).y());
  ptf = this->pixelToMap(ptf);
  ::geometry_msgs::Point32_<std::allocator<void>> point;
  point.x = ptf.x();
  point.y = ptf.y();
  track.edge_points.push_back(point);

  ptf = QPointF(pg.point(1).x(), pg.point(1).y());
  ptf = this->pixelToMap(ptf);
  point.x = ptf.x();
  point.y = ptf.y();
  track.edge_points.push_back(point);

  ptf = QPointF(pg.point(2).x(), pg.point(2).y());
  ptf = this->pixelToMap(ptf);
  point.x = ptf.x();
  point.y = ptf.y();
  track.edge_points.push_back(point);

  ptf = QPointF(pg.point(3).x(), pg.point(3).y());
  ptf = this->pixelToMap(ptf);
  point.x = ptf.x();
  point.y = ptf.y();
  track.edge_points.push_back(point);

  ++ m_nNewTracksCount;
  m_ptfTargets.clear();
}

void QPlanningShowWidget::addGarbageToData()
{
  QPolygonF pgf = this->createTargetPgf(m_ptfTargets, m_ptfTargetMove);
  QPolygon pg = pgf.toPolygon();
  QPointF pt = this->pixelToMap(pg.point(0));
  QPointF pt2 = this->pixelToMap(pg.point(1));
  QPointF pt3 = this->pixelToMap(pg.point(2));
  QLineF linef(pt, pt3);
  QPointF center = linef.center();
  double length = QLineF(pt, pt2).length();
  double width = QLineF(pt2, pt3).length();
  linef = QLineF(0, 0, 1.0, 0);
  QLineF linef2 = QLineF(0, 0, center.x(), center.y());

  auto &garbage_results = m_planningData.garbage_detection_results;
  const auto size = garbage_results.size();

  typedef debug_tool::ads_garbage_detection_<std::allocator<void>> type_garbage;
  type_garbage garbage;
  garbage.id = size > 0 ? garbage_results[size - 1].id + 1 : 0;
  garbage.size = length * width;
  garbage.angle = - linef.angleTo(linef2) * PI / 180.0;
  garbage.distance = qSqrt(center.x() * center.x() + center.y() * center.y());
  garbage.length = length;
  garbage.width = width;

  garbage_results.push_back(garbage);
  ++ m_nNewGarbageCount;
  m_ptfTargets.clear();
}
