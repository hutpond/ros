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
#include <functional>

class QBaseShowWidget : public QWidget
{
  Q_OBJECT

public:
  explicit QBaseShowWidget(QWidget *parent);
  virtual ~QBaseShowWidget();

  void setViewResolution(int);
  void setFunPosition(std::function<void(float, float, float, float)>);

protected:
  virtual void drawImage() = 0;
  virtual void calcMapRect() = 0;

  void drawMapBorder(QPainter &);
  void drawAxis(QPainter &);
  void doUpdate(bool);
  QPointF pixelToMap(const QPointF &);

protected:
  virtual void showEvent(QShowEvent *) override;
  virtual void resizeEvent(QResizeEvent *) override;
  virtual void paintEvent(QPaintEvent *) override;
  virtual void mousePressEvent(QMouseEvent *) override;
  virtual void mouseMoveEvent(QMouseEvent *) override;
  virtual void wheelEvent(QWheelEvent *) override;

protected:
  QImage m_image;
  QRectF m_rectfMap;              // 显示区域的物理范围，单位：米，车体坐标系
  QRect  m_rectPicture;           // 显示区域的像素范围，单位：像素，屏幕显示坐标系

  QTransform m_transform;
  QPointF m_ptfMouseMove;   // 鼠标移动位置记录
  QPointF m_ptfTranslate;   // 移动距离
  float m_fDisplayRatio;    // 缩放比例
  float m_fOriginRatio;     // 原始缩放比例

  std::function<void(float, float, float, float)> m_funPosition;
};

#endif // Q_BASE_SHOW_WIDGET_H
