/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionShow3DWidget.h
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块三维图形描绘类
********************************************************/
#ifndef Q_PERCEPTION_SHOW_3D_WIDGET_H
#define Q_PERCEPTION_SHOW_3D_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions_1_0>

struct PerceptionData;

class QPerceptionShow3DWidget : public QOpenGLWidget
{
  Q_OBJECT

public:
  QPerceptionShow3DWidget(QWidget *parent);
  ~QPerceptionShow3DWidget();

  void setPerceptionData(const PerceptionData *);

protected:
  void mousePressEvent(QMouseEvent *);
  void mouseMoveEvent(QMouseEvent *);
  void wheelEvent(QWheelEvent *);

  void initializeGL();
  void resizeGL(int w, int h);
  void paintGL();

private:
  QVector< QVector3D> m_vctPoints;
  QPointF m_ptfLast;
  GLfloat m_rotationX;
  GLfloat m_rotationY;
  GLfloat m_rotationZ;
  int m_nMouseDelta;
  QOpenGLFunctions_1_0 m_glfun10;
};

#endif // Q_PERCEPTION_SHOW_3D_WIDGET_H
