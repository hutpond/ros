/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QBaseShowWidget.h
 * Author: liuzheng
 * Date: 2019/7/8
 * Description: 图像显示基类
********************************************************/
#ifndef Q_BASE_SHOW_WIDGET_H
#define Q_BASE_SHOW_WIDGET_H

#include <QWidget>
#include <boost/function.hpp>

class QBaseShowWidget : public QWidget
{
  Q_OBJECT

public:
  QBaseShowWidget(QWidget *parent);
  ~QBaseShowWidget();

  void setViewResolution(int);
  void setFunPosition(boost::function<void(float, float, float, float)>);

protected:
  virtual void drawImage() = 0;
  void drawMapBorder(QPainter &);
  void drawAxis(QPainter &);

  virtual void calcMapRect() = 0;

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void paintEvent(QPaintEvent *);
  virtual void mouseMoveEvent(QMouseEvent *);
  virtual void wheelEvent(QWheelEvent *);

protected:
  QImage m_image;
  QRectF m_rectfMap;              // 显示区域的物理范围，单位：米，车体坐标系
  QRect  m_rectPicture;           // 显示区域的像素范围，单位：像素，屏幕显示坐标系

  QTransform m_transform;
  QPointF m_ptfMouseMove;   // 鼠标移动位置记录
  QPointF m_ptfTranslate;   // 移动距离
  float m_fDisplayRatio;    // 缩放比例
  float m_fOriginRatio;     // 原始缩放比例

  boost::function<void(float, float, float, float)> m_funPosition;
};

#endif // Q_BASE_SHOW_WIDGET_H
