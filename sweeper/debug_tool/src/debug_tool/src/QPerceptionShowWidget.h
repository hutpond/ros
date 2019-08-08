/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionShowWidget.h
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块二维图形描绘类
********************************************************/
#ifndef Q_PERCEPTION_SHOW_WIDGET_H
#define Q_PERCEPTION_SHOW_WIDGET_H

#include "QBaseShowWidget.h"

struct PerceptionData;

class QPerceptionShowWidget : public QBaseShowWidget
{
  Q_OBJECT

public:
  QPerceptionShowWidget(QWidget *parent);
  ~QPerceptionShowWidget();

  void setPerceptionData(const PerceptionData *);

protected:
  virtual void mousePressEvent(QMouseEvent *);

protected:
  void drawImage();
  void drawSweeper(QPainter &);
  void drawRoadSide(QPainter &);
  void drawObstacle(QPainter &);
  void drawObstacleOriginal(QPainter &);

  void calcMapRect();

private:
  PerceptionData *m_perceptionData;
};

#endif // Q_PERCEPTION_SHOW_WIDGET_H
