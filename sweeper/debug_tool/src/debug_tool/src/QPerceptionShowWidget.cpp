/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionShowWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块二维图形描绘类
********************************************************/
#include <QPainter>
#include <QMouseEvent>
#include "QPerceptionShowWidget.h"
#include "QPerceptionWidget.h"
#include "GlobalDefine.h"

QPerceptionShowWidget::QPerceptionShowWidget(QWidget *parent)
  : QBaseShowWidget(parent)
{
  m_perceptionData = new PerceptionData;
}

QPerceptionShowWidget::~QPerceptionShowWidget()
{
  delete m_perceptionData;
  m_perceptionData = NULL;
}

void QPerceptionShowWidget::mousePressEvent(QMouseEvent *e)
{
  m_ptfMouseMove = e->localPos();
}

/*******************************************************
 * @brief 根据数据将图像画在QImage上

 * @return
********************************************************/
void QPerceptionShowWidget::drawImage()
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
  this->drawObstacle(painter);
  this->drawObstacleOriginal(painter);
}

/*******************************************************
 * @brief 设置perception数据
 * @param data: perception数据

 * @return
********************************************************/
void QPerceptionShowWidget::setPerceptionData(const PerceptionData *data)
{
  memcpy(&m_perceptionData->m_rectSweeper, &data->m_rectSweeper, sizeof(Rect));
  memcpy(&m_perceptionData->m_roadSide, &data->m_roadSide, sizeof(RoadSide));
  m_perceptionData->m_lidarOriginal.clear();
  m_perceptionData->m_lidarOriginal.assign(data->m_lidarOriginal.begin(),
                                           data->m_lidarOriginal.end());
  m_perceptionData->m_radarOriginal.clear();
  m_perceptionData->m_radarOriginal.assign(data->m_radarOriginal.begin(),
                                           data->m_radarOriginal.end());
  memcpy(m_perceptionData->m_ultrasonic, data->m_ultrasonic, sizeof(data->m_ultrasonic));
  m_perceptionData->m_lidar.clear();
  m_perceptionData->m_lidar.assign(data->m_lidar.begin(),	data->m_lidar.end());
  m_perceptionData->m_radar.clear();
  m_perceptionData->m_radar.assign(data->m_radar.begin(), data->m_radar.end());

  this->calcMapRect();
  this->drawImage();
  this->update();
}

/*******************************************************
 * @brief 绘制扫地车
 * @param painter: 画笔

 * @return
********************************************************/
void QPerceptionShowWidget::drawSweeper(QPainter &painter)
{
  painter.save();
  QPolygonF pgf = m_transform.map(g_rectfSweeper);
  painter.setPen(Qt::red);
  painter.drawPolygon(pgf);
  painter.restore();
}

/*******************************************************
 * @brief 绘制路边沿
 * @param painter: 画笔

 * @return
********************************************************/
void QPerceptionShowWidget::drawRoadSide(QPainter &painter)
{
  QLineF linefLeft(
        m_perceptionData->m_roadSide.m_lineLeft.m_p1.m_fX,
        m_perceptionData->m_roadSide.m_lineLeft.m_p1.m_fY,
        m_perceptionData->m_roadSide.m_lineLeft.m_p2.m_fX,
        m_perceptionData->m_roadSide.m_lineLeft.m_p2.m_fY
        );
  linefLeft = m_transform.map(linefLeft);

  QLineF linefRight(
        m_perceptionData->m_roadSide.m_lineRight.m_p1.m_fX,
        m_perceptionData->m_roadSide.m_lineRight.m_p1.m_fY,
        m_perceptionData->m_roadSide.m_lineRight.m_p2.m_fX,
        m_perceptionData->m_roadSide.m_lineRight.m_p2.m_fY
        );
  linefRight = m_transform.map(linefRight);

  painter.save();
  QPen pen;
  pen.setWidth(4);
  pen.setColor(Qt::yellow);
  pen.setStyle(Qt::SolidLine);
  painter.setPen(pen);

  painter.drawLine(linefLeft);
  painter.drawLine(linefRight);

  painter.restore();
}

void QPerceptionShowWidget::drawObstacle(QPainter &painter)
{
  painter.save();
  QPen pen;

  pen.setColor(Qt::blue);
  pen.setStyle(Qt::SolidLine);
  painter.setPen(pen);
  const int SIZE = static_cast<int>(m_perceptionData->m_lidar.size());
  for (int i = 0; i < SIZE; ++i) {
    const Rect &it = m_perceptionData->m_lidar[i];
    QRectF rectf = QRectF(0, 0, it.m_fLength, it.m_fWidth);
    rectf.moveCenter(QPointF(
                       it.m_fCenterX, it.m_fCenterY
                       ));
    QPolygonF pgf = m_transform.map(rectf);
    if (!qFuzzyIsNull(it.m_fAngle)) {
      QMatrix matrix;
      matrix.rotate(it.m_fAngle * 360 / PI);
      pgf = matrix.map(pgf);
    }

    painter.drawPolygon(pgf);
  }

  pen.setColor(Qt::red);
  painter.setPen(pen);

  const int SIZE_R = static_cast<int>(m_perceptionData->m_radar.size());
  for (int i = 0; i < SIZE_R; ++i) {
    const Rect &it = m_perceptionData->m_radar[i];
    QRectF rectf = QRectF(0, 0, it.m_fLength, it.m_fWidth);
    rectf.moveCenter(QPointF(
                       it.m_fCenterX, it.m_fCenterY
                       ));
    QPolygonF pgf = m_transform.map(rectf);
    if (!qFuzzyIsNull(it.m_fAngle)) {
      QMatrix matrix;
      matrix.rotate(it.m_fAngle * 360 / PI);
      pgf = matrix.map(pgf);
    }
    painter.drawPolygon(pgf);
  }

  painter.restore();
}

void QPerceptionShowWidget::drawObstacleOriginal(QPainter &painter)
{
  painter.save();
  QPen pen;

  pen.setColor(Qt::blue);
  pen.setStyle(Qt::DashLine);
  painter.setPen(pen);
  const int SIZE = static_cast<int>(m_perceptionData->m_lidarOriginal.size());
  for (int i = 0; i < SIZE; ++i) {
    const Rect &it = m_perceptionData->m_lidarOriginal[i];
    QRectF rectf = QRectF(0, 0, it.m_fLength, it.m_fWidth);
    rectf.moveCenter(QPointF(
                       it.m_fCenterX, it.m_fCenterY
                       ));
    QPolygonF pgf = m_transform.map(rectf);
    if (!qFuzzyIsNull(it.m_fAngle)) {
      QMatrix matrix;
      matrix.rotate(it.m_fAngle * 360 / PI);
      pgf = matrix.map(pgf);
    }
    painter.drawPolygon(pgf);
  }

  pen.setColor(Qt::red);
  painter.setPen(pen);

  const int SIZE_R = static_cast<int>(m_perceptionData->m_radarOriginal.size());
  for (int i = 0; i < SIZE_R; ++i) {
    const Rect &it = m_perceptionData->m_radarOriginal[i];
    QRectF rectf = QRectF(0, 0, it.m_fLength, it.m_fWidth);
    rectf.moveCenter(QPointF(
                       it.m_fCenterX, it.m_fCenterY
                       ));
    QPolygonF pgf = m_transform.map(rectf);
    if (!qFuzzyIsNull(it.m_fAngle)) {
      QMatrix matrix;
      matrix.rotate(it.m_fAngle * 360 / PI);
      pgf = matrix.map(pgf);
    }
    painter.drawPolygon(pgf);
  }

  painter.restore();
}

void QPerceptionShowWidget::calcMapRect()
{
  // 计算显示区域物理范围，车体坐标系，X正向：上，Y正向：左，坐标原点：车中心
  // 显示范围，height（Y向）：路宽MAP_TO_ROAD_COEF倍，
  // width（X向）：根据显示区域比例计算，起点：车身后START_X_TO_CAR_TAIL米
  float roadWidth = qAbs<float>(m_perceptionData->m_roadSide.m_lineLeft.m_p1.m_fY -
                                m_perceptionData->m_roadSide.m_lineRight.m_p1.m_fY);
  roadWidth = qMax<float>(g_rectfSweeper.width() * 2, roadWidth);
  const float mapHeight = m_fDisplayRatio * roadWidth;
  const float mapWidth = mapHeight * m_rectPicture.height() / m_rectPicture.width();
  const float mapX = g_rectfSweeper.center().x() - g_rectfSweeper.width() / 2 -
      VEH_HEAD - m_ptfTranslate.x();
  const float roadRightWidth = m_perceptionData->m_roadSide.m_lineRight.m_p1.m_fY;
  const float mapY = roadRightWidth - roadWidth * (m_fDisplayRatio - 1.0) / 2.0
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
