/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QLocusWidget.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 位置轨迹显示画面
********************************************************/
#ifndef Q_LOCUS_WIDGET_H
#define Q_LOCUS_WIDGET_H

#include <QWidget>

class QLocusWidget : public QWidget
{
  Q_OBJECT

public:
  QLocusWidget(QWidget *parent = Q_NULLPTR);
  ~QLocusWidget();

private:
};

#endif // Q_LOCUS_WIDGET_H
