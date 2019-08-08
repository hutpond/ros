/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QUltrasonicWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: 超声波触发状态数据
********************************************************/
#include <QLabel>
#include <QGridLayout>
#include <QDateTime>
#include "QUltrasonicWidget.h"
#include "GlobalDefine.h"
#include "QPerceptionWidget.h"

QUltrasonicWidget::QUltrasonicWidget(QWidget *parent)
  : QWidget(parent)
{
  this->setFont(G_TEXT_FONT);

  QLabel *pLblTitle = new QLabel(QStringLiteral("UltraSonic"), this);
  pLblTitle->setAlignment(Qt::AlignCenter);
  for (int i = 0; i < Count; ++i) {
    m_pLblUltrasonicState[i] = new QLabel(this);
    m_pLblUltrasonicState[i]->setAlignment(Qt::AlignCenter);
  }

  QGridLayout *mainLayout = new QGridLayout;
  mainLayout->addWidget(pLblTitle, 0, 3, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[TopLeft], 1, 1, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[TopRight], 1, 5, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[LeftTop], 2, 0, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[RightTop], 2, 6, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[LeftMiddle], 3, 0, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[RightMiddle], 3, 6, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[LeftBottom], 4, 0, 1, 3);
  mainLayout->addWidget(m_pLblUltrasonicState[RightBottom], 4, 6, 1, 3);
  setLayout(mainLayout);
}

QUltrasonicWidget::~QUltrasonicWidget()
{
}

/*******************************************************
 * @brief 设置超声波状态数据，如果数据发生变化，改变画面状态显示
 * @param data: 超声波状态数据

 * @return
********************************************************/
void QUltrasonicWidget::setData(const Ultrasonic *data)
{
  for (int i = 0; i < Count; ++i) {
    if (data[i].m_nId < 0 || data[i].m_nId >= Count) continue;
    //m_pLblUltrasonicState[i]->setStyleSheet(data[i].m_bTriggered ?
    //	"background-color: rgb(255, 0, 0);" :
    //	"background-color: rgb(255, 255, 0);");
    QPalette pe;
    pe.setColor(QPalette::WindowText, data[i].m_bTriggered ? Qt::red : Qt::black);
    m_pLblUltrasonicState[i]->setPalette(pe);
    QString text = QString("%1:%2").arg(i).arg(data[i].m_fDistance, 3, 'f', 2);
    m_pLblUltrasonicState[i]->setText(text);
  }
}

void QUltrasonicWidget::resizeEvent(QResizeEvent *)
{

}
