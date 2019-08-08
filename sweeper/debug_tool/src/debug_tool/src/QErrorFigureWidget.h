/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QErrorFigureWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 误差图显示画面
********************************************************/
#ifndef Q_ERROR_FIGURE_WIDGET_H
#define Q_ERROR_FIGURE_WIDGET_H

#include <QWidget>

class QErrorFigureWidget : public QWidget
{
  Q_OBJECT

public:
  QErrorFigureWidget(QWidget *parent = Q_NULLPTR);
  ~QErrorFigureWidget();

private:
};

#endif // Q_ERROR_FIGURE_WIDGET_H
