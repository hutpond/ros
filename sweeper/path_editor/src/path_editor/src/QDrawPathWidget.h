/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDrawPathWidget.h
 * Author: liuzheng
 * Date: 2019/7/19
 * Description: 路径绘制
********************************************************/
#ifndef QDRAWPATHWIDGET_H
#define QDRAWPATHWIDGET_H

#include <QWidget>

class QProjectObject;

class QDrawPathWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QDrawPathWidget(QProjectObject &, QWidget *parent = nullptr);
  ~QDrawPathWidget();

protected:
  void resizeEvent(QResizeEvent *);
  void paintEvent(QPaintEvent *);
  void timerEvent(QTimerEvent *);
  void mousePressEvent(QMouseEvent *);
  void mouseReleaseEvent(QMouseEvent *);
  void mouseMoveEvent(QMouseEvent *);
  void wheelEvent(QWheelEvent *e);

protected:
  void drawImage();
  void drawPath(QPainter &);
  void drawMapBorder(QPainter &);
  void drawSelectRect(QPainter &);

  void calcMapRect();

signals:

public slots:
  void onOperate(int);

private:
  QProjectObject &m_rObjProject;

  QImage m_image;
  QRectF m_rectfMap;              // 显示区域的物理范围，单位：米，车体坐标系
  QRect  m_rectPicture;           // 显示区域的像素范围，单位：像素，屏幕显示坐标系
  QRectF m_rectfSelect;            // 鼠标选择区域像素范围
  QRectF m_rectfSelectMap;        // 鼠标选择区域物理范围

  QTransform m_transform;
  QPointF m_ptfMouseMove;   // 鼠标移动位置记录
  QPointF m_ptfTranslate;   // 移动距离
  int m_nDisplayRatio;      // 缩放比例

  int m_nTimerId;
  int m_nOperateIndex;
  bool m_bMouseLeftPressed;
  int m_nDeleteRadius;
};

#endif // QDRAWPATHWIDGET_H
