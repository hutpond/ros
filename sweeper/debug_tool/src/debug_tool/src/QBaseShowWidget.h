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
#include "debug_tool/ads_PlanningData4Debug.h"
#include "debug_ads_msgs/ads_msgs_planning_debug_frame.h"

class QBaseShowWidget : public QWidget
{
  Q_OBJECT

public:
  enum {
    NEW_COST,
    OLD_COST
  };

  enum {
    EnuCoord,
    FrenetCoord
  };


public:
  QBaseShowWidget(QWidget *parent);
  ~QBaseShowWidget();

  void setCostType(int);
  void setViewResolution(int);
  void setFunPosition(boost::function<void(float, float, float, float)>);

  virtual void setShowAllTargets(bool);
  virtual void setToolIndex(int, bool) {}
  virtual void changeShowCoord() {}
  virtual int showCoord() {return 0;}

  virtual void setPlanningData(const debug_tool::ads_PlanningData4Debug &) {}
  virtual void setPlanningData(const debug_ads_msgs::ads_msgs_planning_debug_frame &) {}

protected:
  void doUpdate(bool);
  virtual void drawImage() = 0;
  void drawMapBorder(QPainter &);
  void drawAxis(QPainter &);

  virtual void calcMapRect() = 0;

  QPointF pixelToMap(const QPointF &);
  QPolygonF createTargetPgf(const QVector<QPointF> &, const QPointF &);
  void addTargetMouseMove(QMouseEvent *);

protected:
  virtual void showEvent(QShowEvent *);
  virtual void resizeEvent(QResizeEvent *);
  virtual void paintEvent(QPaintEvent *);
  virtual void mouseMoveEvent(QMouseEvent *);
  virtual void wheelEvent(QWheelEvent *);

signals:
  void saveDataToFile(const debug_tool::ads_PlanningData4Debug &);

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
  int m_nCostType;
  bool m_bFlagShowAllTargets;
  int m_nToolIndex;

  QVector<QPointF> m_ptfTargets;
  QPointF m_ptfTargetMove;
};

#endif // Q_BASE_SHOW_WIDGET_H
