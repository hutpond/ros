/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPanelWidget.h
 * Author: liuzheng
 * Date: 2019/7/19
 * Description: 路径编辑面板
********************************************************/
#ifndef QPANNELWIDGET_H
#define QPANNELWIDGET_H

#include <QWidget>

class QPushButton;

class QPanelWidget : public QWidget
{
  Q_OBJECT

public:
  enum
  {
    TypeMove,
    TypeDelete,
    TypeZoomLocal,
    TypeZoomIn,
    TypeZoomOut,
    TypeZoomReset,
    TypeCount
  };
public:
  explicit QPanelWidget(QWidget *parent = nullptr);

signals:
  void operate(int);

private slots:
  void onBtnSlot();

private:
  QPushButton *m_pBtnOperate[TypeCount];
  int m_nBtnCheckedIndex;
};

#endif // QPANNELWIDGET_H
