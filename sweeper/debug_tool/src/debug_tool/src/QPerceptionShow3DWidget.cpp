/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionShow3DWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块三维图形描绘类
********************************************************/
#include <cmath>
#include <QVector3D>
#include <QDateTime>
#include <QMouseEvent>
#include "QPerceptionShow3DWidget.h"

QPerceptionShow3DWidget::QPerceptionShow3DWidget(QWidget *parent)
  : QOpenGLWidget(parent)
  , m_nMouseDelta(0)
{
#ifdef TEST
  qsrand(QDateTime::currentDateTime().toSecsSinceEpoch());
  for (int i = 0; i < 1000; ++i) {
    int val = qrand();
    float x = static_cast<float>(val % 1500 - 750) / 100;
    val = qrand();
    float y = static_cast<float>(val % 1500 - 750) / 100;
    val = qrand();
    float z = static_cast<float>(val % 500) / 100;
    m_vctPoints.append(QVector3D(x, y, z));
  }
#endif
}

QPerceptionShow3DWidget::~QPerceptionShow3DWidget()
{
}

void QPerceptionShow3DWidget::mousePressEvent(QMouseEvent *e)
{
  m_ptfLast = e->localPos();
}

void QPerceptionShow3DWidget::mouseMoveEvent(QMouseEvent *e)
{
  QPointF ptf = e->localPos();

  int width = this->rect().width();
  int height = this->rect().height();
  int length = std::sqrt(width * width + height * height);
  float dx = ptf.x() - m_ptfLast.x();
  float dy = ptf.y() - m_ptfLast.y();
  float dz = 0;
  if (!qFuzzyCompare(dx * dy, 0)) {
    dz = sqrt(dx * dx + dy * dy) * (qAbs<float>(dx * dy) / (dx * dy));
  }
  m_rotationX = 360 * dx / width;
  m_rotationY = 360 * dy / height;
  m_rotationZ = 360 * dz / length;

  m_ptfLast = ptf;
  this->update();
}

void QPerceptionShow3DWidget::wheelEvent(QWheelEvent *e)
{
  m_nMouseDelta = e->delta() / 120;
  this->update();
}

void QPerceptionShow3DWidget::initializeGL()
{
  m_glfun10.initializeOpenGLFunctions();

  m_glfun10.glClearColor(0, 0, 0, 1);

  m_glfun10.glEnable(GL_DEPTH_TEST);
  m_glfun10.glEnable(GL_CULL_FACE);
  m_glfun10.glOrtho(-8, 8, -2, 6, -8, 8);
}

void QPerceptionShow3DWidget::resizeGL(int, int)
{
}

void QPerceptionShow3DWidget::paintGL()
{
  // Clear color and depth buffer
  m_glfun10.glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  m_glfun10.glRotatef(m_rotationX, 1.0f, 0.0f, 0.0f);
  m_glfun10.glRotatef(m_rotationY, 0.0f, 1.0f, 0.0f);
  m_glfun10.glRotatef(m_rotationZ, 0.0f, 0.0f, 1.0f);

  if (m_nMouseDelta != 0) {
    float scale = std::pow(1.1, m_nMouseDelta);
    m_glfun10.glScalef(scale, scale, scale);
    m_nMouseDelta = 0;
  }

  m_glfun10.glPointSize(2);
  m_glfun10.glBegin(GL_POINTS);
  for (int i = 0; i < m_vctPoints.size(); ++i) {
    m_glfun10.glVertex3f(m_vctPoints[i].x(), m_vctPoints[i].z(), m_vctPoints[i].y());
  }
  m_glfun10.glEnd();
  m_glfun10.glPointSize(1);
}

void QPerceptionShow3DWidget::setPerceptionData(const PerceptionData *)
{

}
