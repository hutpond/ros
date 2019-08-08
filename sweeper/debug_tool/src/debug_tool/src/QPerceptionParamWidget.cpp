/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPerceptionParamWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/26
 * Description: Perception模块参数显示、设定类
********************************************************/
#include <QGridLayout>
#include <QLabel>
#include <QGroupBox>
#include <QPushButton>
#include <QSlider>
#include "QPerceptionParamWidget.h"
#include "QUltrasonicWidget.h"
#include "GlobalDefine.h"
#include "QPerceptionWidget.h"

QPerceptionParamWidget::QPerceptionParamWidget(QWidget *parent)
  : QWidget(parent)
{
  this->setFont(G_TEXT_FONT);
  m_pWdgUltrasonic = new QUltrasonicWidget(this);

  QVBoxLayout *mainLayout = new QVBoxLayout;

  QGridLayout *layoutLegend = new QGridLayout;
  QLabel *pLabel = new QLabel(tr("图例"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignCenter);
  layoutLegend->addWidget(pLabel, 0, 2, 1, 1);

  pLabel = new QLabel(tr("车辆"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignCenter);
  pLabel->setStyleSheet("border:2px solid blue,");
  layoutLegend->addWidget(pLabel, 1, 1, 1, 1);

  pLabel = new QLabel(tr("行人"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignCenter);
  pLabel->setStyleSheet("border:2px solid magenta,");
  layoutLegend->addWidget(pLabel, 1, 3, 1, 1);

  pLabel = new QLabel(tr("Rider"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignCenter);
  pLabel->setStyleSheet("border:2px solid cyan,");
  layoutLegend->addWidget(pLabel, 2, 1, 1, 1);

  pLabel = new QLabel(tr("NotFilter"));
  layoutLegend->addWidget(pLabel, 2, 3, 1, 1);
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignCenter);
  pLabel->setStyleSheet("border:2px dashed green,");
  mainLayout->addLayout(layoutLegend);
  mainLayout->setStretchFactor(layoutLegend, 3);

  QGridLayout *layoutState = new QGridLayout;
  pLabel = new QLabel(tr("CIPV类型"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(pLabel, 0, 0, 1, 2);

  m_pLblCipvType = new QLabel(this);
  m_pLblCipvType->setFont(G_TEXT_FONT);
  m_pLblCipvType->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(m_pLblCipvType, 0, 2, 1, 1);

  pLabel = new QLabel(tr("CIPV距离"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(pLabel, 1, 0, 1, 2);

  m_pLblCipvDistance = new QLabel(this);
  m_pLblCipvDistance->setFont(G_TEXT_FONT);
  m_pLblCipvDistance->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(m_pLblCipvDistance, 1, 2, 1, 1);

  pLabel = new QLabel(tr("路宽"));
  pLabel->setFont(G_TEXT_FONT);
  pLabel->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(pLabel, 2, 0, 1, 2);

  m_pLblRoadSide = new QLabel();
  pLabel->setFont(G_TEXT_FONT);
  m_pLblRoadSide->setAlignment(Qt::AlignVCenter | Qt::AlignLeft);
  layoutState->addWidget(m_pLblRoadSide, 2, 2, 1, 5);
  mainLayout->addLayout(layoutState);
  mainLayout->setStretchFactor(layoutState, 3);
  mainLayout->addStretch(2);

  mainLayout->addWidget(m_pWdgUltrasonic);
  mainLayout->setStretchFactor(m_pWdgUltrasonic, 5);

  m_pGBoxReplay = new QGroupBox(this);
  QGridLayout *layout = new QGridLayout;
  m_pBtnPause = new QPushButton(tr("暂停"), m_pGBoxReplay);
  connect(m_pBtnPause, &QPushButton::clicked, this, &QPerceptionParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnPause, 0, 1, 1, 1);
  m_pBtnResume = new QPushButton(tr("恢复"), m_pGBoxReplay);
  connect(m_pBtnResume, &QPushButton::clicked, this, &QPerceptionParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnResume, 0, 2, 1, 1);
  m_pBtnBack = new QPushButton(tr("后退"), m_pGBoxReplay);
  connect(m_pBtnBack, &QPushButton::clicked, this, &QPerceptionParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnBack, 0, 3, 1, 1);
  m_pBtnNext = new QPushButton(tr("前进"), m_pGBoxReplay);
  connect(m_pBtnNext, &QPushButton::clicked, this, &QPerceptionParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnNext, 0, 4, 1, 1);
  m_pBtnData = new QPushButton(tr("数据"), m_pGBoxReplay);
  connect(m_pBtnData, &QPushButton::clicked, this, &QPerceptionParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnData, 0, 5, 1, 1);
  m_pSliderPlay = new QSlider(Qt::Horizontal, m_pGBoxReplay);
  connect(m_pSliderPlay, &QSlider::valueChanged,
          this, &QPerceptionParamWidget::onSliderValueChanged);
  layout->addWidget(m_pSliderPlay, 1, 1, 1, 5);
  m_pGBoxReplay->setLayout(layout);
  mainLayout->addWidget(m_pGBoxReplay);
  mainLayout->setStretchFactor(m_pGBoxReplay, 3);

  setLayout(mainLayout);

  connect(m_pBtnData, &QPushButton::clicked, this, &QPerceptionParamWidget::displayData);
}

QPerceptionParamWidget::~QPerceptionParamWidget()
{
}

/*******************************************************
 * @brief 设置CIPV数据
 * @param index: CIPV类型
 * @param distance: CIPV距离

 * @return
********************************************************/
void QPerceptionParamWidget::setCipvData(int index, float distance)
{
  QString strType;
  switch (index) {
    case 0:
      strType = tr("Car");
      break;
    default:
      break;
  }
  m_pLblCipvType->setText(strType);
  QString strDistance = QString("%1").arg(distance, 4, 'f', 2, QLatin1Char(' '));
  m_pLblCipvDistance->setText(strDistance);
}

/*******************************************************
 * @brief 设置路边沿距离
 * @param left: 左路边沿距离
 * @param right: 右路边沿距离

 * @return
********************************************************/
void QPerceptionParamWidget::setRoadSide(float left, float right)
{
  QString text = QString("W:%1(L:%2, R:%3)").
      arg(left + right, 4, 'f', 2, QLatin1Char(' ')).
      arg(left, 4, 'f', 2, QLatin1Char(' ')).
      arg(right, 4, 'f', 2, QLatin1Char(' '));
  m_pLblRoadSide->setText(text);
}

/*******************************************************
 * @brief 设置perception数据
 * @param data: perception数据

 * @return
********************************************************/
void QPerceptionParamWidget::setPerceptionData(const PerceptionData *data)
{
  this->setRoadSide(qAbs<float>(data->m_roadSide.m_lineLeft.m_p1.m_fY),
                    qAbs<float>(data->m_roadSide.m_lineRight.m_p1.m_fY));
  m_pWdgUltrasonic->setData(data->m_ultrasonic);
}

/*******************************************************
 * @brief 回放button点击响应槽函数
 * @param

 * @return
********************************************************/
void QPerceptionParamWidget::onBtnClicked()
{
  QObject *sender = this->sender();
  if (sender == m_pBtnPause) {
    m_pBtnPause->setEnabled(false);
    m_pBtnResume->setEnabled(true);
    m_pBtnBack->setEnabled(true);
    m_pBtnNext->setEnabled(true);
    m_pBtnData->setEnabled(true);
    m_pSliderPlay->setEnabled(true);
    emit replayState(true);
  }
  else if (sender == m_pBtnResume) {
    m_pBtnPause->setEnabled(true);
    m_pBtnResume->setEnabled(false);
    m_pBtnBack->setEnabled(false);
    m_pBtnNext->setEnabled(false);
    m_pBtnData->setEnabled(false);
    m_pSliderPlay->setEnabled(false);
    emit replayState(false);
  }
  else if (sender == m_pBtnBack) {
    this->setFrameOffset(-1);
    emit replayFrameOffset(-1);
  }
  else if (sender == m_pBtnNext) {
    this->setFrameOffset(1);
    emit replayFrameOffset(1);
  }
  else if (sender == m_pBtnData) {
  }
}

/*******************************************************
 * @brief slider值改变响应槽函数
 * @param value: slider改变后的值

 * @return
********************************************************/
void QPerceptionParamWidget::onSliderValueChanged(int value)
{
  if (m_nSliderValue != value) {
    emit replayFrameOffset(value - m_nSliderValue);
    m_nSliderValue = value;
  }
}


/*******************************************************
 * @brief 设置显示类型：Live Display & Replay
 * @param index: 显示类型

 * @return
********************************************************/
void QPerceptionParamWidget::setShowType(int index)
{
  if (index == QBaseWidget::LiveDisplay) {
    m_pGBoxReplay->hide();
  }
  else if (index == QBaseWidget::Replay) {
    m_nSliderValue = 0;
    m_pGBoxReplay->show();
    m_pBtnPause->setEnabled(true);
    m_pBtnResume->setEnabled(false);
    m_pBtnBack->setEnabled(false);
    m_pBtnNext->setEnabled(false);
    m_pBtnData->setEnabled(false);
    m_pSliderPlay->setEnabled(false);
  }
}

/*******************************************************
 * @brief 设置数据总帧数，用于设置拖动条范围
 * @param count: 数据总帧数

 * @return
********************************************************/
void QPerceptionParamWidget::setFrameCount(int count)
{
  m_pSliderPlay->setMinimum(0);
  m_pSliderPlay->setMaximum(count);
  m_pSliderPlay->setPageStep(1);
  m_pSliderPlay->setSingleStep(1);
}

/*******************************************************
 * @brief 设置slider改变量
 * @param int: 改变量

 * @return
********************************************************/
void QPerceptionParamWidget::setFrameOffset(int offset)
{
  int nValue = m_nSliderValue + offset;
  m_nSliderValue = qBound<int>(m_pSliderPlay->minimum(), nValue, m_pSliderPlay->maximum());
  m_pSliderPlay->setValue(m_nSliderValue);
}
