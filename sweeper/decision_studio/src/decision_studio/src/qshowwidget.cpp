#include "qshowwidget.h"

#include <QPainter>
#include "QBezierCurve.h"

QShowWidget::QShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
{

}

void QShowWidget::setData(const decision_studio::ads_DecisionData4Debug &data)
{
  m_decisionData = data;
  QBaseShowWidget::doUpdate(true);
}

void QShowWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(210, 210, 210));

  QBaseShowWidget::drawAxis(painter);
  QBaseShowWidget::drawMapBorder(painter);

  this->drawRoadSideFromWidth(painter);
  this->drawSweeper(painter);
  this->drawTrackTargetWithPoints(painter);
}

void QShowWidget::calcMapRect()
{
  const int size_reference_splines = m_decisionData.reference_splines.size();
  double left_road_width = 5;
  double right_road_width = 3;
  if (size_reference_splines == 0) {
    left_road_width = 5;
    right_road_width = 3;
    m_decisionData.front_vehicle_length = 2.2;
  }
  else {
    left_road_width = m_decisionData.reference_points[0].left_road_width;
    right_road_width = m_decisionData.reference_points[0].right_road_width;
  }

  // 计算显示区域物理范围，车体坐标系，X正向：上，Y正向：左，坐标原点：车中心
  // 显示范围，height（Y向）：路宽MAP_TO_ROAD_COEF倍，
  // width（X向）：根据显示区域比例计算，起点：车身后START_X_TO_CAR_TAIL米
  const float VEH_HEAD = m_decisionData.head_point.x;
  const float roadLeftWidth = left_road_width;
  const float roadRightWidth = right_road_width;
  const float roadWidth = roadLeftWidth + roadRightWidth;
  const float mapHeight = m_fDisplayRatio * roadWidth;
  const float mapWidth = mapHeight * m_rectPicture.height() / m_rectPicture.width();
  const float mapX = -(m_decisionData.front_vehicle_length - VEH_HEAD)
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

/*******************************************************
 * @brief 绘制扫地车
 * @param painter: 画笔

 * @return
********************************************************/
void QShowWidget::drawSweeper(QPainter &painter)
{
  painter.save();
  QPen pen;

  // front of vehicle
  const double VEH_W = m_decisionData.front_vehicle_width;
  const double VEH_L = m_decisionData.front_vehicle_length;
  const double VEH_HEAD = m_decisionData.head_point.x;

  QRectF rectfSweeper = QRectF(-VEH_L + VEH_HEAD, -VEH_W / 2, VEH_L, VEH_W);
  QPolygonF pgfSweeper = m_transform.map(rectfSweeper);
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawPolygon(pgfSweeper);

  // front line
  QLineF linef(m_decisionData.front_axle_center.x, m_decisionData.front_axle_center.y,
               m_decisionData.hinge_point.x, m_decisionData.hinge_point.y);
  linef = m_transform.map(linef);
  pen.setColor(Qt::green);
  pen.setWidth(2);
  painter.setPen(pen);
  painter.drawLine(linef);

  // back line
  linef = QLineF(m_decisionData.hinge_point.x, m_decisionData.hinge_point.y,
                 m_decisionData.rear_axle_center.x, m_decisionData.rear_axle_center.y
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
  linef = QLineF(m_decisionData.hinge_point.x, m_decisionData.hinge_point.y,
                 m_decisionData.rear_axle_center.x, m_decisionData.rear_axle_center.y
               );

  QPolygonF pgf;
  ptf = QPointF(m_decisionData.rear_point.x, m_decisionData.rear_point.y);
  QLineF linef2;
  linef2.setP1(ptf);
  linef2.setLength(m_decisionData.rear_vehicle_width / 2.0);
  linef2.setAngle(linef.normalVector().angle());
  pgf << linef2.p2();
  QPointF ptfStart = linef2.p2();
  linef2.setAngle(linef.normalVector().angle() + 180);
  pgf << linef2.p2();

  linef2 = QLineF(m_decisionData.rear_point.x, m_decisionData.rear_point.y,
                 m_decisionData.hinge_point.x, m_decisionData.hinge_point.y
               );
  linef2.setLength(m_decisionData.rear_vehicle_length);
  ptf = linef2.p2();

  linef2.setP2(QPointF(1000000.0, 0));
  linef2.setP1(ptf);
  linef2.setLength(m_decisionData.rear_vehicle_width / 2.0);
  linef2.setAngle(linef.normalVector().angle() + 180);
  pgf << linef2.p2();
  linef2.setAngle(linef.normalVector().angle());
  pgf << linef2.p2();
  pgf << ptfStart;

  pgf = m_transform.map(pgf);
  painter.drawPolyline(pgf);

  painter.restore();

  //this->drawRadar(painter);
  //this->drawUltrasonic(painter);
}


/**
 * @brief 绘制参考线，路边沿(根据参考线左侧、右侧路宽数据绘制）
 * @param painter
 */
void QShowWidget::drawRoadSideFromWidth(QPainter &painter)
{
  painter.save();

  // reference
  QPen pen;
  pen.setWidth(1);
  pen.setColor(Qt::black);
  pen.setStyle(Qt::DashLine);
  painter.setPen(pen);

  for (const auto &spline : m_decisionData.reference_splines) {
    QPainterPath path(QPointF(spline.xb.x, spline.yb.x));
    path.cubicTo(
          spline.xb.y, spline.yb.y,
          spline.xb.z, spline.yb.z,
          spline.xb.w, spline.yb.w
          );
    painter.drawPath(m_transform.map(path));
  }

  // reference points size
  const auto &points = m_decisionData.reference_points;
  const int size_ref_points = points.size();
  if (size_ref_points == 0) {
    painter.restore();
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
 * @brief 绘制激光障碍物，路边沿以内，以角点画图
 * @param painter: 画笔

 * @return
********************************************************/
void QShowWidget::drawTrackTargetWithPoints(QPainter &painter)
{
  const auto &TRACKS = m_decisionData.fusion_results;
  const int SIZE = static_cast<int>(m_decisionData.fusion_results.size());
  const int SIZE_REF = static_cast<int>(m_decisionData.reference_points.size());

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

    pgf = m_transform.map(pgf);
    painter.drawPolygon(pgf);
    painter.drawText(pgf.boundingRect(), Qt::AlignCenter, QString::number(i));
  }

  painter.restore();
}

/**
 * @brief 根据s坐标查找位于参考点的区域范围
 * @param s: s坐标系

 * @return -1: 起点之外, SIZE - 1: 终点之外, 其它: index, index+1之间
*/
int QShowWidget::findReferenceIndex(const double s)
{
  const auto &REF_PTS = m_decisionData.reference_points;
  const int SIZE = m_decisionData.reference_points.size();
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
int QShowWidget::findReferenceIndex(const double x, const double y)
{
  int index = -1;
  double distance = 100000;

  const auto &points = m_decisionData.reference_points;
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
void QShowWidget::xyToSl(const QPointF &ptfXy, double &s, double &l)
{
  const auto &REF_PTS = m_decisionData.reference_points;
  const int SIZE = m_decisionData.reference_points.size();
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
void QShowWidget::slToXy(const double s, const double l, QPointF &ptfXy)
{
  const auto &REF_PTS = m_decisionData.reference_points;
  const int SIZE = m_decisionData.reference_points.size();

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
