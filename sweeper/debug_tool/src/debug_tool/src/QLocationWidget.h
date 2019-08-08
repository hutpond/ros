/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QLocationWidget.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 融合定位模块显示画面
********************************************************/
#ifndef Q_LOCATION_WIDGET_H
#define Q_LOCATION_WIDGET_H

#include <QWidget>
//#include "ui_QLocationWidget.h"

class QLocationWidget : public QWidget
{
  Q_OBJECT

public:
  QLocationWidget(QWidget *parent = Q_NULLPTR);
  ~QLocationWidget();

private:
//  Ui::QLocationWidget ui;


};

#endif // Q_LOCATION_WIDGET_H
