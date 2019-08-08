/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QUltrasonicWidget.h
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: 超声波触发状态数据
********************************************************/
#ifndef Q_ULTRASONIC_WIDGET_H
#define Q_ULTRASONIC_WIDGET_H

#include <QWidget>

class QLabel;
struct Ultrasonic;

class QUltrasonicWidget : public QWidget
{
  Q_OBJECT

public:
  enum
  {
    TopLeft = 0,
    TopRight,
    LeftTop,
    LeftMiddle,
    LeftBottom,
    RightTop,
    RightMiddle,
    RightBottom,
    Count
  };

public:
  QUltrasonicWidget(QWidget *parent);
  ~QUltrasonicWidget();

  void setData(const Ultrasonic *);

protected:
  virtual void resizeEvent(QResizeEvent *);

private:
  //Ultrasonic m_ultrasonicData[Count];
  QLabel *m_pLblUltrasonicState[Count];
};

#endif // Q_ULTRASONIC_WIDGET_H
