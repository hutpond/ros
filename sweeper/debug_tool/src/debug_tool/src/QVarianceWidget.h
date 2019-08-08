/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QVarianceWidget.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 融合数据信息方差显示画面
********************************************************/
#ifndef Q_VARIANCE_WIDGET_H
#define Q_VARIANCE_WIDGET_H

#include <QWidget>

class QVarianceWidget : public QWidget
{
  Q_OBJECT

public:
  QVarianceWidget(QWidget *parent = Q_NULLPTR);
  ~QVarianceWidget();

private:
};

#endif // Q_VARIANCE_WIDGET_H
