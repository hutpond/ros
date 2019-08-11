/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningParamWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划参数子画面，显示相关参数和设定显示选项
********************************************************/
#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <QGridLayout>
#include <QSlider>
#include "QPlanningParamWidget.h"
#include "QPlanningWidget.h"
#include "GlobalDefine.h"

QPlanningParamWidget::QPlanningParamWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pLblRoadWidthName = new QLabel(tr("路宽"), this);
  m_pLblRoadWidthValue = new QLabel(this);
  m_pLblDecisionName = new QLabel(tr("Decision"), this);
  m_pLblDecisionValue = new QLabel(this);
  m_pLblPlanningPtName = new QLabel(tr("规划点SL"), this);
  m_pLblPlanningPtValue = new QLabel(this);

  m_pLblMousePosName = new QLabel(tr("鼠标坐标"), this);
  m_pLblMousePosValue = new QLabel(this);

  m_pLblCarSizeName = new QLabel(tr("车体大小"), this);
  m_pLblCarSizeValue = new QLabel(this);
  m_pLblCarEnuName = new QLabel(tr("车体ENU"), this);
  m_pLblCarEnuValue = new QLabel(this);
  m_pLblCarSLName = new QLabel(tr("车体SL"), this);
  m_pLblCarSLValue = new QLabel(this);

  m_pGBoxReplay = new QGroupBox(this);
  QGridLayout *layout = new QGridLayout;
  m_pBtnPause = new QPushButton(tr("暂停"), m_pGBoxReplay);
  connect(m_pBtnPause, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnPause, 0, 1, 1, 1);
  m_pBtnResume = new QPushButton(tr("恢复"), m_pGBoxReplay);
  connect(m_pBtnResume, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnResume, 0, 2, 1, 1);
  m_pBtnBack = new QPushButton(tr("后退"), m_pGBoxReplay);
  connect(m_pBtnBack, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnBack, 0, 3, 1, 1);
  m_pBtnNext = new QPushButton(tr("前进"), m_pGBoxReplay);
  connect(m_pBtnNext, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnNext, 0, 4, 1, 1);
  m_pBtnData = new QPushButton(tr("数据"), m_pGBoxReplay);
  connect(m_pBtnData, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);
  layout->addWidget(m_pBtnData, 0, 5, 1, 1);
  m_pSliderPlay = new QSlider(Qt::Horizontal, m_pGBoxReplay);
  connect(m_pSliderPlay, &QSlider::valueChanged,
          this, &QPlanningParamWidget::onSliderValueChanged);
  layout->addWidget(m_pSliderPlay, 1, 1, 1, 5);
  m_pGBoxReplay->setLayout(layout);

  connect(m_pBtnData, &QPushButton::clicked, this, &QPlanningParamWidget::displayData);
}

QPlanningParamWidget::~QPlanningParamWidget()
{
}

/*******************************************************
 * @brief 设置显示类型：Live Display & Replay
 * @param index: 显示类型

 * @return
********************************************************/
void QPlanningParamWidget::setShowType(int index)
{
  if (index == QPlanningWidget::LiveDisplay) {
    m_pGBoxReplay->hide();
  }
  else if (index == QPlanningWidget::Replay) {
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
void QPlanningParamWidget::setFrameCount(int count)
{
  m_pSliderPlay->setMinimum(0);
  m_pSliderPlay->setMaximum(count);
  m_pSliderPlay->setPageStep(1);
  m_pSliderPlay->setSingleStep(1);
}

void QPlanningParamWidget::setPlanningData(const debug_tool::PlanningData4Debug &data)
{
  // 路宽值
  const float fRoadWidthLeft = data.left_half_road_width;
  const float fRoadWidthRight = data.right_half_road_width;
  QString strText = QString("W:%1 (L:%2, R:%3)").
      arg(fRoadWidthLeft + fRoadWidthRight, 6, 'f', 2).
      arg(fRoadWidthLeft, 6, 'f', 2).
      arg(fRoadWidthRight, 6, 'f', 2);
  m_pLblRoadWidthValue->setText(strText);

  // 决策状态
  int nIndex = static_cast<int>(data.decision);
  int nIndexUs = static_cast<int>(data.ultrasonic_decision);
  int nIndexRadar = static_cast<int>(data.radar28f_decision);
  int nIndexRadar73 = static_cast<int>(data.radar73f_decision);
  m_pLblDecisionValue->setFont(G_TEXT_FONT);
  strText = QString("%1 [us: %2 r28: %3 r73: %4]").arg(this->getDecisionText(nIndex)).
      arg(this->getDecisionText(nIndexUs)).
      arg(this->getDecisionText(nIndexRadar)).
      arg(this->getDecisionText(nIndexRadar73));
  m_pLblDecisionValue->setText(strText);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue->setStyleSheet("background-color: rgb(0, 0, 0, 0);");
  }
  else {
    m_pLblDecisionValue->setStyleSheet("background-color: rgb(255, 0, 0);");
  }

  // 规划点位置
  const float fPlanningS = data.trajectory_end_s;
  const float fPlanningL = data.trajectory_end_l;
  strText = QString("S:%1 L:%2").
      arg(fPlanningS, 6, 'f', 2).
      arg(fPlanningL, 6, 'f', 2);
  m_pLblPlanningPtValue->setText(strText);

  // 车体
  const float fCarWidth = data.vehicle_width;
  const float fCarLength = data.vehicle_length;
  strText = QString("%1:%2 %3:%4").
      arg(tr("Width")).
      arg(fCarWidth, 6, 'f', 2).
      arg(tr("Length")).
      arg(fCarLength, 6, 'f', 2);
  m_pLblCarSizeValue->setText(strText);
  // ENU
  const float fCarE = data.vehicle_x;
  const float fCarN = data.vehicle_y;
  const float fCarU = data.vehicle_z;
  strText = QString("%1 %2 %3").
      arg(fCarE, 6, 'f', 2).
      arg(fCarN, 6, 'f', 2).
      arg(fCarU, 6, 'f', 2);
  m_pLblCarEnuValue->setText(strText);
  // SL
  const float fCarS = data.vehicle_s;
  const float fCarL = data.vehicle_l;
  strText = QString("S:%1 L:%2").
      arg(fCarS, 6, 'f', 2).
      arg(fCarL, 6, 'f', 2);
  m_pLblCarSLValue->setText(strText);

  // 障碍
}

QString QPlanningParamWidget::getDecisionText(int index)
{
  QString strText;
  switch (index)
  {
    case -1:
      strText = tr("quit");
      break;
    case 0:
      strText = tr("Forward");
      break;
    case 1:
      strText = tr("Left Pass");
      break;
    case 2:
      strText = tr("Right Pass");
      break;
    case 3:
      strText = tr("Follow");
      break;
    case 4:
      strText = tr("Safe Stop");
      break;
    default:
      strText = tr("Unknow");
      break;
  }
  return strText;
}

/*******************************************************
 * @brief 显示鼠标点处物理坐标的回调函数
 * @param s: s坐标
 * @param l: l坐标

 * @return
********************************************************/
void QPlanningParamWidget::showMousePosition(float x, float y, float s, float l)
{
  QString strText = QString("X:%1, Y:%2, S:%3, L%4").
      arg(x, 6, 'f', 2).
      arg(y, 6, 'f', 2).
      arg(s, 6, 'f', 2).
      arg(l, 6, 'f', 2);
  m_pLblMousePosValue->setText(strText);
}

/*******************************************************
 * @brief 设置slider改变量
 * @param int: 改变量

 * @return
********************************************************/
void QPlanningParamWidget::setFrameOffset(int offset)
{
  int nValue = m_nSliderValue + offset;
  m_nSliderValue = qBound<int>(m_pSliderPlay->minimum(), nValue, m_pSliderPlay->maximum());
  m_pSliderPlay->setValue(m_nSliderValue);
}

/*******************************************************
 * @brief 回放button点击响应槽函数
 * @param

 * @return
********************************************************/
void QPlanningParamWidget::onBtnClicked()
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
void QPlanningParamWidget::onSliderValueChanged(int value)
{
  if (m_nSliderValue != value) {
    emit replayFrameOffset(value - m_nSliderValue);
    m_nSliderValue = value;
  }
}

void QPlanningParamWidget::resizeEvent(QResizeEvent *)
{
  const float F_SPACE_X_P = 0.01f;
  const float F_SPACE_Y_P = 0.01f;
  const float F_ITEM_H_P = 0.032f;
  const float F_ITEM_NAME_W_P = 0.2f;

  const int WIDTH = this->width();
  const int HEIGHT = this->height();
  const int SPACE_X = WIDTH * F_SPACE_X_P;
  const int SPACE_Y = HEIGHT * F_SPACE_Y_P;
  const int ITEM_NAME_W = WIDTH * F_ITEM_NAME_W_P;
  const int ITEM_VALUE_W = WIDTH - ITEM_NAME_W - 3 * SPACE_X;
  const int ITEM_H = HEIGHT * F_ITEM_H_P;

  int xPos = SPACE_X;
  int yPos = SPACE_Y;
  m_pLblRoadWidthName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblRoadWidthValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblDecisionName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblDecisionValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblPlanningPtName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblPlanningPtValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblMousePosName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblMousePosValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  yPos += ITEM_H + SPACE_Y;
  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblCarSizeName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblCarSizeValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblCarEnuName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblCarEnuValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pLblCarSLName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblCarSLValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  yPos += ITEM_H + SPACE_Y;
  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pGBoxReplay->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H * 8);
}
