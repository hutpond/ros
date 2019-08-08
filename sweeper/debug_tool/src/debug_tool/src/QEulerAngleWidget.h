/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QEulerAngleWidget.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 姿态角显示画面
********************************************************/
#ifndef Q_EULER_ANGLE_WIDGET_H
#define Q_EULER_ANGLE_WIDGET_H

#include <QWidget>

class QEulerAngleWidget : public QWidget
{
  Q_OBJECT

public:
  QEulerAngleWidget(QWidget *parent = Q_NULLPTR);
  ~QEulerAngleWidget();

private:
};

#endif // Q_EULER_ANGLE_WIDGET_H
