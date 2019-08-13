/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPanelWidget.cpp
 * Author: liuzheng
 * Date: 2019/7/19
 * Description: 路径编辑面板
********************************************************/
#include <QPushButton>
#include <QVBoxLayout>
#include "QPanelWidget.h"

QPanelWidget::QPanelWidget(QWidget *parent)
  : QWidget(parent)
{
  constexpr int BTN_SIZE = 40;
  QVBoxLayout *layout = new QVBoxLayout;
  for (int i = 0; i < TypeCount; ++i) {
    m_pBtnOperate[i] = new QPushButton(this);
    m_pBtnOperate[i]->setMinimumSize(BTN_SIZE, BTN_SIZE);
    layout->addWidget(m_pBtnOperate[i], 1);
    connect(m_pBtnOperate[i], &QPushButton::clicked, this, &QPanelWidget::onBtnSlot);
  }
  m_pBtnOperate[TypeMove]->setIcon(QIcon(":/image/hand_checked.svg"));
  m_pBtnOperate[TypeDelete]->setIcon(QIcon(":/image/delete.svg"));
  m_pBtnOperate[TypeZoomIn]->setIcon(QIcon(":/image/zoomin.svg"));
  m_pBtnOperate[TypeZoomOut]->setIcon(QIcon(":/image/zoomout.svg"));
  m_pBtnOperate[TypeZoomLocal]->setIcon(QIcon(":/image/local.svg"));
  m_pBtnOperate[TypeZoomReset]->setIcon(QIcon(":/image/reset.svg"));
  m_nBtnCheckedIndex = TypeMove;

  for (int i = 0; i < TypeCount; ++i) {
    m_pBtnOperate[i]->setIconSize(QSize(BTN_SIZE, BTN_SIZE));
  }

  this->setLayout(layout);
}

void QPanelWidget::onBtnSlot()
{
  int index = -1;
  QObject *sender = this->sender();
  for (int i = 0; i < TypeCount; ++i) {
    if (sender == m_pBtnOperate[i]) {
      index = i;
      break;
    }
  }
  if (index == -1) {
    return;
  }
  switch (index) {
    case TypeMove:
      m_pBtnOperate[index]->setIcon(QIcon(":/image/hand_checked.svg"));
      break;
    case TypeDelete:
      m_pBtnOperate[index]->setIcon(QIcon(":/image/delete_checked.svg"));
      break;
    case TypeZoomLocal:
      m_pBtnOperate[index]->setIcon(QIcon(":/image/local_checked.svg"));
      break;
    case TypeZoomIn:

      break;
    case TypeZoomOut:

      break;
    case TypeZoomReset:

      break;
    default:
      break;
  }
  if (m_nBtnCheckedIndex != index && index != TypeZoomIn && index != TypeZoomOut
      && index != TypeZoomReset) {
    switch (m_nBtnCheckedIndex) {
      case TypeMove:
        m_pBtnOperate[m_nBtnCheckedIndex]->setIcon(QIcon(":/image/hand.svg"));
        m_nBtnCheckedIndex = index;
        break;
      case TypeDelete:
        m_pBtnOperate[m_nBtnCheckedIndex]->setIcon(QIcon(":/image/delete.svg"));
        m_nBtnCheckedIndex = index;
        break;
      case TypeZoomLocal:
        m_pBtnOperate[m_nBtnCheckedIndex]->setIcon(QIcon(":/image/local.svg"));
        m_nBtnCheckedIndex = index;
        break;
      default:
        break;
    }
  }

  emit operate(index);
}
