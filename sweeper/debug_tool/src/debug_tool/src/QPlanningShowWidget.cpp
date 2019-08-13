/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningShowWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划显示子画面，可以实时显示和回放
********************************************************/
#include <cmath>
#include <QPainter>
#include <QMouseEvent>
#include <QDebug>
#include <QThread>
#include "QPlanningShowWidget.h"
#include "GlobalDefine.h"

QPlanningShowWidget::QPlanningShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
  , m_nShowPlanningPath(0)
{
}

QPlanningShowWidget::~QPlanningShowWidget()
{
}

void QPlanningShowWidget::mousePressEvent(QMouseEvent *e)
{
  m_ptfMouseMove = e->localPos();
  QTransform inverted = m_transform.inverted();
  QPointF ptf = inverted.map(m_ptfMouseMove);
  double s, l;
  this->xyToSl(ptf, s, l);
  m_funPosition(ptf.x(), ptf.y(), s, l);
}

/*******************************************************
 * @brief 设置扫规划数据
 * @param data: 规划数据

 * @return
********************************************************/
void QPlanningShowWidget::setPlanningData(const debug_tool::PlanningData4Debug &data)
{
  m_planningData = data;
  //memcpy(&m_planningData, &data, sizeof(data));
  this->calcMapRect();
  this->drawImage();
  this->update();
}

/*******************************************************
 * @brief 计算显示区域，车体坐标系

 * @return
********************************************************/
void QPlanningShowWidget::calcMapRect()
{
  // 计算显示区域物理范围，车体坐标系，X正向：上，Y正向：左，坐标原点：车中心
  // 显示范围，height（Y向）：路宽MAP_TO_ROAD_COEF倍，
  // width（X向）：根据显示区域比例计算，起点：车身后START_X_TO_CAR_TAIL米
  const float roadLeftWidth = m_planningData.left_half_road_width;
  const float roadRightWidth = m_planningData.right_half_road_width;
  const float roadWidth = roadLeftWidth + roadRightWidth;
  const float mapHeight = m_fDisplayRatio * roadWidth;
  const float mapWidth = mapHeight * m_rectPicture.height() / m_rectPicture.width();
  const float mapX = -(m_planningData.vehicle_length - VEH_HEAD)
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
 * @brief 设置显示鼠标点处物理坐标的回调函数
 * @param fun: 回调函数

 * @return
********************************************************/
void QPlanningShowWidget::setFunPosition(boost::function<void(float, float, float, float)> fun)
{
  m_funPosition = fun;
}

/*******************************************************
 * @brief 缩放图像显示
 * @param index: -1， 缩小, 0, 复原, 1, 放大

 * @return
********************************************************/
void QPlanningShowWidget::setViewResolution(int index)
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
 * @brief 根据数据将图像画在QImage上

 * @return
********************************************************/
void QPlanningShowWidget::drawImage()
{
  m_image = QImage(m_rectPicture.width(), m_rectPicture.height(), QImage::Format_RGB888);
  QPainter painter(&m_image);
  painter.fillRect(m_image.rect(), QColor(230, 230, 230));

#ifdef TEST
  this->drawMapBorder(painter);
#endif
  this->drawRoadSide(painter);
  this->drawSweeper(painter);
  this->drawAxis(painter);
  this->drawAreaLine(painter);
  this->drawPlanningPoint(painter);
  this->drawPlanningPath(painter);
  this->drawDecisionTargets(painter);
  this->drawTrackTarget(painter);
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

  // vehicle
  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;
  QRectF rectfSweeper = QRectF(-VEH_L + VEH_HEAD, -VEH_W / 2, VEH_L, VEH_W);
  QPolygonF pgfSweeper = m_transform.map(rectfSweeper);
  pen.setColor(Qt::red);
  painter.setPen(pen);
  painter.drawPolygon(pgfSweeper);

  // stop line
  const double STOP_DIS = 0.5;
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
  painter.drawPolygon(pgf);

  this->drawUltrasonic(painter);
  this->drawRadar(painter);

  painter.restore();
}

void QPlanningShowWidget::drawUltrasonic(QPainter &painter)
{
  painter.save();

  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;

  QPen pen;
  const int US_W = 13;
  QRect rectf(0, 0, US_W, US_W);
  pen.setStyle(Qt::SolidLine);
  painter.setBrush(Qt::darkBlue);
  painter.setFont(QFont("Times", 10));
  QPointF ptf;

  ptf = QPointF(-VEH_L + VEH_HEAD + 0.13, VEH_W / 2);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "8");

  ptf.setY(ptf.y() - VEH_W);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "12");

  ptf.setX(ptf.x() + 0.97);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "13");

  ptf.setY(ptf.y() + VEH_W);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "9");

  ptf.setX(ptf.x() + 0.98);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "10");

  ptf.setY(ptf.y() - VEH_W);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "14");

  ptf = QPointF(-VEH_L + VEH_HEAD + 0.13, VEH_W / 2);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "8");

  const double HEAD_US = 0.05;
  ptf = QPointF(VEH_HEAD + HEAD_US, VEH_W / 2 + HEAD_US);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "11");

  ptf = QPointF(VEH_HEAD + HEAD_US, -VEH_W / 2 - HEAD_US);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawEllipse(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "15");

  painter.restore();
}

void QPlanningShowWidget::drawRadar(QPainter &painter)
{
  painter.save();

  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;

  QPen pen;
  const int US_W = 11;
  QRect rectf(0, 0, US_W, US_W * 2);
  pen.setStyle(Qt::SolidLine);
  painter.setBrush(Qt::darkBlue);
  painter.setFont(QFont("Times", 10));
  QPointF ptf;

  ptf = QPointF(-VEH_L + VEH_HEAD + 0.7, VEH_W / 2);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  rectf.moveLeft(rectf.left() - US_W / 2);
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawRect(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "2");

  ptf.setY(ptf.y() - VEH_W);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  rectf.moveLeft(rectf.left() + US_W / 2);
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawRect(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "3");

  ptf.setX(ptf.x() + 0.8);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  rectf.moveLeft(rectf.left() + US_W / 2);
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawRect(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "1");

  ptf.setY(ptf.y() + VEH_W);
  rectf.moveCenter(m_transform.map(ptf).toPoint());
  rectf.moveLeft(rectf.left() - US_W / 2);
  pen.setColor(Qt::darkBlue);
  painter.setPen(pen);
  painter.drawRect(rectf);
  pen.setColor(Qt::white);
  painter.setPen(pen);
  painter.drawText(rectf, Qt::AlignCenter, "0");

  painter.restore();
}

/*******************************************************
 * @brief 绘制路边沿、参考线，路边沿平行于参考线
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawRoadSide(QPainter &painter)
{
  const double leftWidth = m_planningData.left_half_road_width;
  const double rightWidth = m_planningData.right_half_road_width;
  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);

  QPointF ptfLeft, ptfRight;
  QPolygonF pgfLeft, pgfRight, pgfReference;
  for (int i = 0; i < SIZE; ++i) {
    this->slToXy(m_planningData.reference_points[i].s, leftWidth, ptfLeft);
    this->slToXy(m_planningData.reference_points[i].s, - rightWidth, ptfRight);

    pgfLeft << ptfLeft;
    pgfRight << ptfRight;
    pgfReference << QPointF(m_planningData.reference_points[i].x,
                            m_planningData.reference_points[i].y);

    m_ptfsLeftRoadSide[i] = ptfLeft;
    m_ptfsRightRoadSide[i] = ptfRight;
  }

  painter.save();

  QPen pen;
  pen.setWidth(4);
  pen.setColor(Qt::yellow);
  pen.setStyle(Qt::SolidLine);
  painter.setPen(pen);
  painter.drawPolyline(m_transform.map(pgfLeft));
  painter.drawPolyline(m_transform.map(pgfRight));

  pen.setWidth(1);
  pen.setColor(Qt::black);
  pen.setStyle(Qt::DashLine);
  painter.setPen(pen);
  painter.drawPolyline(m_transform.map(pgfReference));

  painter.restore();
}

/*******************************************************
 * @brief 绘制区域分界线，分界线垂直于分界线
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawAreaLine(QPainter &painter)
{
  const double AREA_LINE[3] = {
    m_planningData.safe_dis1,
    m_planningData.safe_dis2,
    m_planningData.max_planning_distance
  };

  // 计算分界线起始点、终点
  QLineF linefArea[3];
  const double fLeftWidth = m_planningData.left_half_road_width * 1.2;
  const double fRightWidth = m_planningData.right_half_road_width * 1.2;
  for (int i = 0; i < 3; ++i) {
    QPointF ptfLeft, ptfRight;
    this->slToXy(AREA_LINE[i] + m_planningData.head_distance, fLeftWidth, ptfLeft);
    this->slToXy(AREA_LINE[i] + m_planningData.head_distance, -fRightWidth, ptfRight);
    linefArea[i] = QLineF(ptfLeft, ptfRight);
  }

  // 绘制分界线
  painter.save();
  QPen pen;
  pen.setStyle(Qt::DashLine);
  pen.setColor(Qt::black);
  painter.setFont(G_TEXT_SMALL_FONT);
  painter.setPen(pen);

  for (int i = 0; i < 3; ++i) {
    QLineF linef = m_transform.map(linefArea[i]);
    painter.drawLine(linef);
    painter.drawText(
          linef.p2() + QPoint(10, 5),
          QString("%1").arg(AREA_LINE[i], 5, 'f', 2, QLatin1Char(' '))
          );
  }
  painter.restore();
}

/*******************************************************
 * @brief 绘制决策点
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawDecisionPoint(QPainter &painter)
{
  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &pts = m_planningData.reference_points;

  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);
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
  QPointF ptfPlanning = QPointF(
        m_planningData.trajectory_end_x,
        m_planningData.trajectory_end_y
        );
  QPointF ptf = m_transform.map(ptfPlanning);
  QRect rect = QRect(0, 0, 7, 7);
  rect.moveCenter(ptf.toPoint());
  QLineF line(rect.topLeft(), rect.bottomRight());
  QLineF line2(rect.bottomLeft(), rect.topRight());

  painter.save();
  QPen pen;
  pen.setColor(Qt::blue);
  pen.setWidth(2);
  painter.setPen(pen);
  painter.drawLine(line);
  painter.drawLine(line2);
  painter.restore();
}

/*******************************************************
 * @brief 绘制规划路径，包括当前帧和前一帧
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawPlanningPath(QPainter &painter)
{
  if (m_planningData.decision == 4) {
    return;
  }

  painter.save();
  const int SIZE = 40;
  // 当前帧
  double coef[5] = {0};
  coef[0] = m_planningData.trajectory_b0;
  coef[1] = m_planningData.trajectory_b1;
  coef[2] = m_planningData.trajectory_b2;
  coef[3] = m_planningData.trajectory_b3;
  coef[4] = m_planningData.trajectory_b4;
  QPen pen;
  pen.setWidth(2);
  pen.setColor(Qt::blue);
  painter.setPen(pen);
  const double dLenX = static_cast<double>(m_planningData.trajectory_end_x -
                                           m_planningData.trajectory_start_x) / SIZE;
  const double dStartX = static_cast<double>(m_planningData.trajectory_start_x);
  for (int i = 1; i <= SIZE; ++i) {
    const double fX = dStartX + (i - 1) * dLenX;
    const double fX2 = dStartX + i * dLenX;
    double fY = 0;
    double fY2 = 0;
    for (int j = 0; j < 5; ++j) {
      fY += coef[j] * pow(fX, j);
      fY2 += coef[j] * pow(fX2, j);
    }
    QLineF linef(fX, fY, fX2, fY2);
    linef = m_transform.map(linef);
    painter.drawLine(linef);
  }

  // 三次曲线
  coef[0] = m_planningData.trajectory_a0;
  coef[1] = m_planningData.trajectory_a1;
  coef[2] = m_planningData.trajectory_a2;
  coef[3] = m_planningData.trajectory_a3;
  pen.setColor(Qt::magenta);
  painter.setPen(pen);
  const double dLenS = static_cast<double>(m_planningData.trajectory_end_s -
                                           m_planningData.trajectory_start_s) / SIZE;
  const double dStartS = static_cast<double>(m_planningData.trajectory_start_s);
  for (int i = 1; i <= SIZE; ++i) {
    // calc s, l
    const double fS = dStartS + (i - 1) * dLenS;
    const double fS2 = dStartS + i * dLenS;
    double fL = 0;
    double fL2 = 0;
    for (int j = 0; j < 4; ++j) {
      fL += coef[j] * pow(fS, j);
      fL2 += coef[j] * pow(fS2, j);
    }
    // calc x,y
    int index = -1, index2 = -1;
    const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);
    for (int j = 0; j < SIZE - 1; ++j) {
      const debug_tool::ReferencePoint &pointPre = m_planningData.reference_points[j];
      const debug_tool::ReferencePoint &point = m_planningData.reference_points[j + 1];
      if (index == -1 && fS >= pointPre.s && fS < point.s) {
        index = j;
      }
      if (index2 == -1 && fS2 >= pointPre.s && fS2 < point.s) {
        index2 = j;
      }
      if (index > 0 && index2 > 0) {
        break;
      }
    }
    if (index == -1 || index2 == -1) {
      continue;
    }
    const debug_tool::ReferencePoint &ptStart = m_planningData.reference_points[index];
    const debug_tool::ReferencePoint &ptStart2 = m_planningData.reference_points[index + 1];
    QLineF linefStart;
    float fSDis = (ptStart2.s - ptStart.s);
    if ( qAbs<float>(fS - ptStart.s) < qAbs<float>(ptStart2.s - fS) ) {
      linefStart = QLineF(ptStart.x, ptStart.y, ptStart2.x, ptStart2.y);
      QPointF ptfStart = linefStart.pointAt( (fS - ptStart.s) / fSDis );
      linefStart.setP1(ptfStart);
      linefStart = linefStart.normalVector();
    }
    else {
      linefStart = QLineF(ptStart2.x, ptStart2.y, ptStart.x, ptStart.y);
      QPointF ptfStart = linefStart.pointAt( (ptStart2.s - fS) / fSDis );
      linefStart.setP1(ptfStart);
//      linefStart = QLineF(linefStart.p2(), linefStart.p1());
      linefStart = linefStart.normalVector().normalVector().normalVector();
    }
    linefStart.setLength(-fL);

    const debug_tool::ReferencePoint &ptEnd = m_planningData.reference_points[index2];
    const debug_tool::ReferencePoint &ptEnd2 = m_planningData.reference_points[index2 + 1];
    QLineF linefEnd;
    fSDis = (ptEnd2.s - ptEnd.s);
    if ( qAbs<float>(fS2 - ptEnd.s) < qAbs<float>(ptEnd2.s - fS2) ) {
      linefEnd = QLineF(ptEnd.x, ptEnd.y, ptEnd2.x, ptEnd2.y);
      QPointF ptfEnd = linefEnd.pointAt( (fS2 - ptEnd.s) / fSDis );
      linefEnd.setP1(ptfEnd);
      linefEnd = linefEnd.normalVector();
    }
    else {
      linefEnd = QLineF(ptEnd2.x, ptEnd2.y, ptEnd.x, ptEnd.y);
      QPointF ptfEnd = linefEnd.pointAt( (ptEnd2.s - fS2) / fSDis );
      linefEnd.setP1(ptfEnd);
//      linefEnd = QLineF(linefEnd.p2(), linefEnd.p1());
      linefEnd = linefEnd.normalVector().normalVector().normalVector();
    }
    linefEnd.setLength(-fL2);

    QLineF linef(linefStart.p2(), linefEnd.p2());
    linef = m_transform.map(linef);
    painter.drawLine(linef);
  }

  {
    QPointF ptfEnd;
    this->slToXy(m_planningData.trajectory_end_s, m_planningData.trajectory_end_l,
                 ptfEnd);
    ptfEnd = m_transform.map(ptfEnd);
    QRect rect = QRect(0, 0, 7, 7);
    rect.moveCenter(ptfEnd.toPoint());
    QLineF line(rect.topLeft(), rect.bottomRight());
    QLineF line2(rect.bottomLeft(), rect.topRight());

    pen.setWidth(2);
    painter.drawLine(line);
    painter.drawLine(line2);
  }

  painter.restore();
}

/*******************************************************
 * @brief 绘制障碍物
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawObstacleObject(QPainter &painter)
{
  painter.save();
  QPen pen;
  pen.setColor(Qt::red);
  pen.setWidth(2);
  painter.setPen(pen);
  /*const debug_tool::TargetPoint &left = m_planningData.left_target_point;
  if (left.width > 0) {
    QPolygonF pgf = this->createSlPgf(
          QPointF(left.s, left.l), left.width, left.length, true);
    pgf = m_transform.map(pgf);
    painter.drawPolygon(pgf);
  }
  const debug_tool::TargetPoint &right = m_planningData.right_target_point;
  if (right.width > 0) {
    QPolygonF pgf = this->createSlPgf(
          QPointF(right.s, right.l), right.width, right.length, true);
    pgf = m_transform.map(pgf);
    painter.drawPolygon(pgf);
  }*/
  painter.restore();
}

void QPlanningShowWidget::drawUltrasonicTarget(QPainter &painter)
{
  painter.save();

  const debug_tool::UltraSonicTargetColl &ultrasonic = m_planningData.ultrasonic_results;

  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;

  const double STOP_DIS = 0.5;
  const double PASS_DIS = 1.1;
  const int SIZE = static_cast<int>(ultrasonic.object_count);
  for (int i = 0; i < SIZE; ++i) {
    const int ID = static_cast<int>(ultrasonic.us_objects[i].radar_pos_id);
    const double DIS = ultrasonic.us_objects[i].distance;
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
void QPlanningShowWidget::drawRadar28Target(QPainter &painter)
{
  painter.save();

  const debug_tool::Radar28fTargetColl &radar = m_planningData.radar28f_results;

  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;

  const double STOP_DIS = 0.5;
  const double PASS_DIS = 1.1;
  const int SIZE = static_cast<int>(radar.object_count);
  for (int i = 0; i < SIZE; ++i) {
    const int ID = static_cast<int>(radar.radar_objects[i].devid);
    const double DIS_X = radar.radar_objects[i].range_lat;
    const double DIS_Y = radar.radar_objects[i].range_lon;
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
 * @brief 绘制雷达障碍物，路边沿以内
 * @param painter: 画笔

 * @return
********************************************************/
void QPlanningShowWidget::drawRadar73Target(QPainter &painter)
{
  painter.save();

  const debug_tool::Radar73fTargetColl &radar = m_planningData.radar73f_results;

  const double VEH_W = m_planningData.vehicle_width;
  const double VEH_L = m_planningData.vehicle_length;

  const double STOP_DIS = 0.5;
  const double PASS_DIS = 1.1;
  const int SIZE = static_cast<int>(radar.object_count);
  for (int i = 0; i < SIZE; ++i) {
    const int ID = static_cast<int>(radar.radar_objects[i].devid);
    const double DIS_X = radar.radar_objects[i].range_lat;
    const double DIS_Y = radar.radar_objects[i].range_lon;
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
  typedef boost::array< ::debug_tool::TrackTarget_<std::allocator<void>> , 250>  \
      _track_objects_type;
  const _track_objects_type &TRACKS = m_planningData.fusion_results.track_objects;
  const int SIZE = static_cast<int>(m_planningData.fusion_results.object_count);
  const int SIZE_REF = static_cast<int>(m_planningData.num_reference_points);

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
      for (int j = 0; j < SIZE_REF - 1; ++j) {
        if (indexLeft != -1 && indexRight != -1) {
          break;
        }
        if (m_ptfsLeftRoadSide[j].x() > 10 || m_ptfsRightRoadSide[j].x() > 10) {
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

void QPlanningShowWidget::drawDecisionTargets(QPainter &painter)
{
  painter.save();
  for (int i = 0; i < 6; ++ i) {
    debug_tool::TargetPoint_<std::allocator<void>> &targets =
        m_planningData.decision_targets[i];

    switch (targets.sensor_type) {
      case SENSOR_ULTRASONIC:
        {
          QPointF ptf = QPointF(targets.x, targets.y);
          ptf = m_transform.map(ptf);
          QRect rect = QRect(0, 0, 13, 13);
          rect.moveCenter(ptf.toPoint());
          QLine line(rect.topLeft(), rect.bottomRight());
          QLine line2(rect.bottomLeft(), rect.topRight());

          QPen pen;
          pen.setColor("black");
          pen.setWidth(2);
          painter.setPen(pen);
          painter.drawLine(line);
          painter.drawLine(line2);
        }
        break;
      case SENSOR_RADAR_28F:
      case SENSER_RADAR_73F:
        {
          QPointF ptf = QPointF(targets.x, targets.y);
          ptf = m_transform.map(ptf);
          QRect rect = QRect(0, 0, 13, 13);
          rect.moveCenter(ptf.toPoint());
          QLine line(rect.topLeft(), rect.bottomRight());
          QLine line2(rect.bottomLeft(), rect.topRight());

          QPen pen;
          pen.setColor("green");
          pen.setWidth(2);
          painter.setPen(pen);
          painter.drawLine(line);
          painter.drawLine(line2);
        }
        break;
      case SENSOR_TRACK_TARGET:
        {
          QPolygonF pgf;
          pgf << QPointF(targets.p1_x, targets.p1_y)
              << QPointF(targets.p2_x, targets.p2_y)
              << QPointF(targets.p3_x, targets.p3_y)
              << QPointF(targets.p4_x, targets.p4_y);
          pgf = m_transform.map(pgf);

          QPen pen;
          pen.setColor("red");
          pen.setWidth(2);
          painter.setPen(pen);
          painter.drawPolygon(pgf);

          this->drawDecisionTargetsSL(targets, painter);
        }
        break;
      default:
        break;
    }
  }
  painter.restore();
}

void QPlanningShowWidget::drawDecisionTargetsSL(
    const debug_tool::TargetPoint_<std::allocator<void>> &targets,
    QPainter &painter)
{
  painter.save();

  QPointF ptf1, ptf2, ptf3, ptf4;
  double s, l;

  s = targets.s - targets.sl_length / 2;
  l = targets.l - targets.sl_width / 2;
  this->slToXy(s, l, ptf1);

  s = targets.s - targets.sl_length / 2;
  l = targets.l + targets.sl_width / 2;
  this->slToXy(s, l, ptf2);

  s = targets.s + targets.sl_length / 2;
  l = targets.l + targets.sl_width / 2;
  this->slToXy(s, l, ptf3);

  s = targets.s + targets.sl_length / 2;
  l = targets.l - targets.sl_width / 2;
  this->slToXy(s, l, ptf4);

  int indexStart = this->findReferenceIndex(targets.s - targets.sl_length / 2);
  int indexEnd = this->findReferenceIndex(targets.s + targets.sl_length / 2);
  QPointF ptfInter;
  QLineF::IntersectType type = QLineF(ptf1, ptf2).intersect(QLineF(ptf3, ptf4), &ptfInter);
  QPen pen;
  pen.setColor("red");
  pen.setWidth(2);
  pen.setStyle(Qt::DotLine);
  painter.setPen(pen);

  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &REF_PTS = m_planningData.reference_points;
  QPolygonF pgf, pgf2;
  QPointF ptf;

  if (type == QLineF::BoundedIntersection) {
    painter.drawLine(m_transform.map(ptf1), m_transform.map(ptf3));
    painter.drawLine(m_transform.map(ptf4), m_transform.map(ptf2));

    pgf << m_transform.map(ptf2);
    pgf2 << m_transform.map(ptf1);
    for (int i = indexStart + 1; i < indexEnd; ++i) {
      this->slToXy(REF_PTS[i].s, targets.l - targets.sl_width / 2, ptf);
      pgf2 << m_transform.map(ptf);

      this->slToXy(REF_PTS[i].s, targets.l + targets.sl_width / 2, ptf);
      pgf << m_transform.map(ptf);
    }
    pgf << m_transform.map(ptf3);
    pgf2 << m_transform.map(ptf4);
    painter.drawPolyline(pgf);
    painter.drawPolyline(pgf2);
  }
  else {
    painter.drawLine(m_transform.map(ptf1), m_transform.map(ptf2));
    painter.drawLine(m_transform.map(ptf3), m_transform.map(ptf4));

    pgf << m_transform.map(ptf1);
    pgf2 << m_transform.map(ptf2);
    for (int i = indexStart + 1; i < indexEnd; ++i) {
      this->slToXy(REF_PTS[i].s, targets.l - targets.sl_width / 2, ptf);
      pgf << m_transform.map(ptf);
      this->slToXy(REF_PTS[i].s, targets.l + targets.sl_width / 2, ptf);
      pgf2 << m_transform.map(ptf);
    }
    pgf << m_transform.map(ptf4);
    pgf2 << m_transform.map(ptf3);
    painter.drawPolyline(pgf);
    painter.drawPolyline(pgf2);
  }

  painter.restore();
}

/**
 * @brief 根据s坐标查找位于参考点的区域范围
 * @param s: s坐标系

 * @return -1: 起点之外, SIZE - 1: 终点之外, 其它: index, index+1之间
*/
int QPlanningShowWidget::findReferenceIndex(const double s)
{
  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &REF_PTS = m_planningData.reference_points;
  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);
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
 * @brief xy坐标转换为sl坐标
 * @param ptfXy: 车体坐标系，x, 车头正向, y, 车左侧
 * @param s: sl坐标系, s, 参考线切线方向
 * @param s: sl坐标系, l, 参考线垂直向左

 * @return
*/
void QPlanningShowWidget::xyToSl(const QPointF &ptfXy, double &s, double &l)
{
  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &REF_PTS = m_planningData.reference_points;
  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points),100);
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
  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &REF_PTS = m_planningData.reference_points;
  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);

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
  typedef boost::array< ::debug_tool::ReferencePoint_<std::allocator<void>> , 100> \
      _reference_points_type;
  const _reference_points_type &REF_PTS = m_planningData.reference_points;
  const int SIZE = qBound<int>(0, static_cast<int>(m_planningData.num_reference_points), 100);

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
