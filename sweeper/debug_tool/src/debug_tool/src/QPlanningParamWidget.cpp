/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningParamWidget.cpp
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划参数子画面，显示相关参数和设定显示选项
********************************************************/
#include "QPlanningParamWidget.h"

#include <QLabel>
#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <QGridLayout>
#include <QSlider>
#include <QTextBrowser>
#include "QPlanningWidget.h"
#include "GlobalDefine.h"
#include "QCostValueWidget.h"

QPlanningParamWidget::QPlanningParamWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pLblMousePosName = new QLabel(tr("坐标"), this);
  m_pLblMousePosValue = new QLabel(this);

  m_pLblDecisionName = new QLabel(tr("决策"), this);
  for (int i = 0; i < DecisionCount; ++i) {
    m_pLblDecisionValue[i] = new QLabel(this);
    m_pLblDecisionValue[i]->setFont(G_TEXT_FONT);
    m_pLblDecisionValue[i]->setMargin(3);
    m_pLblDecisionValue[i]->setAlignment(Qt::AlignLeft | Qt::AlignVCenter);
  }

  m_pBtnPause = new QPushButton(tr("暂停"), this);
  connect(m_pBtnPause, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

  m_pBtnResume = new QPushButton(tr("恢复"), this);
  connect(m_pBtnResume, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

  m_pBtnBack = new QPushButton(tr("后退"), this);
  connect(m_pBtnBack, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

  m_pBtnNext = new QPushButton(tr("前进"), this);
  connect(m_pBtnNext, &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

  m_pSliderPlay = new QSlider(Qt::Horizontal, this);
  connect(m_pSliderPlay, &QSlider::valueChanged,
          this, &QPlanningParamWidget::onSliderValueChanged);

  m_pWdgCostValue = new QCostValueWidget(this);
  connect(m_pWdgCostValue, &QCostValueWidget::costValueChanged,
          this, &QPlanningParamWidget::costValueChanged);
  //m_pTextBrowser = new QTextBrowser(this);
}

QPlanningParamWidget::~QPlanningParamWidget()
{
}

/*******************************************************
 * @brief 设置显示类型：Live Display & Replay
 * @param

 * @return
********************************************************/
void QPlanningParamWidget::setShowType(int type)
{
  if (type == QPlanningWidget::LivePlay) {
    m_pBtnPause->setVisible(false);
    m_pBtnResume->setVisible(false);
    m_pBtnBack->setVisible(false);
    m_pBtnNext->setVisible(false);
    m_pSliderPlay->setVisible(false);
  }
  else {
    m_pBtnPause->setVisible(true);
    m_pBtnResume->setVisible(true);
    m_pBtnBack->setVisible(true);
    m_pBtnNext->setVisible(true);
    m_pSliderPlay->setVisible(true);

    m_pBtnPause->setEnabled(true);
    m_pBtnResume->setEnabled(false);
    m_pBtnBack->setEnabled(false);
    m_pBtnNext->setEnabled(false);
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

  m_pSliderPlay->setValue(0);
  m_nSliderValue = 0;
}

void QPlanningParamWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data,
                                           const debug_tool::ads_PlanningData4Debug &/*data_cost*/)
{
  // 决策状态
  int nIndex = static_cast<int>(data.decision);
  m_pLblDecisionValue[DecisionAll]->setText(this->getDecisionText(nIndex));

  nIndex = static_cast<int>(data.ultrasonic_decision);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue[DecisionUltraSonic]->hide();
  }
  else {
    m_pLblDecisionValue[DecisionUltraSonic]->setText(tr("超声波"));
    m_pLblDecisionValue[DecisionUltraSonic]->show();
  }

  nIndex = static_cast<int>(data.radar_decision);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue[DecisionRadar73]->hide();
  }
  else {
    m_pLblDecisionValue[DecisionRadar73]->setText(tr("雷达"));
    m_pLblDecisionValue[DecisionRadar73]->show();
  }

  nIndex = static_cast<int>(data.track_target_decision);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue[DecisionTrack]->hide();
  }
  else {
    m_pLblDecisionValue[DecisionTrack]->setText(tr("障碍物"));
    m_pLblDecisionValue[DecisionTrack]->show();
  }

  // data
  //m_pTextBrowser->setText(this->createTrajectoryString(data, data_cost));
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
  m_nSliderValue = qBound<int>(m_pSliderPlay->minimum(),
                                      nValue, m_pSliderPlay->maximum());
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
    m_pSliderPlay->setEnabled(true);
    emit replayState(true);
  }
  else if (sender == m_pBtnResume) {
    m_pBtnPause->setEnabled(true);
    m_pBtnResume->setEnabled(false);
    m_pBtnBack->setEnabled(false);
    m_pBtnNext->setEnabled(false);
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
  const float F_ITEM_H_P = 0.048f;
  const float F_ITEM_NAME_W_P = 0.15f;

  const int WIDTH = this->width();
  const int HEIGHT = this->height();
  const int SPACE_X = WIDTH * F_SPACE_X_P;
  const int SPACE_Y = HEIGHT * F_SPACE_Y_P;
  const int ITEM_NAME_W = WIDTH * F_ITEM_NAME_W_P;
  const int ITEM_VALUE_W = WIDTH - ITEM_NAME_W - 3 * SPACE_X;
  const int ITEM_H = HEIGHT * F_ITEM_H_P;

  int xPos = SPACE_X;
  int yPos = SPACE_Y;
  m_pLblMousePosName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblMousePosValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  const int size_btn = 4;
  const int BTN_W = (WIDTH - SPACE_X * (size_btn + 1))  / size_btn;

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;

  m_pLblDecisionName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblDecisionValue[DecisionAll]->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);
  for (int i = 1; i < DecisionCount; ++i) {
    yPos += ITEM_H + SPACE_Y;
    m_pLblDecisionValue[i]->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);
  }

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pBtnPause->setGeometry(xPos, yPos, BTN_W, ITEM_H);
  xPos += BTN_W + SPACE_X;
  m_pBtnResume->setGeometry(xPos, yPos, BTN_W, ITEM_H);
  xPos += BTN_W + SPACE_X;
  m_pBtnBack->setGeometry(xPos, yPos, BTN_W, ITEM_H);
  xPos += BTN_W + SPACE_X;
  m_pBtnNext->setGeometry(xPos, yPos, BTN_W, ITEM_H);

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pSliderPlay->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H);

  yPos += ITEM_H + SPACE_Y;
  m_pWdgCostValue->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H * 8);

//  xPos = SPACE_X;
//  yPos += ITEM_H * 8 + SPACE_Y;
//  m_pTextBrowser->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H * 3);
}

void QPlanningParamWidget::showReplayControls(bool show)
{
  m_pBtnPause->setHidden(!show);
  m_pBtnResume->setHidden(!show);
  m_pBtnBack->setHidden(!show);
  m_pBtnNext->setHidden(!show);
  m_pSliderPlay->setHidden(!show);
}

QString QPlanningParamWidget::createTrajectoryString(
    const debug_tool::ads_PlanningData4Debug &data,
    const debug_tool::ads_PlanningData4Debug &data_cost)
{
  double value[QPlanningCostWidget::Count];
  QCostValueWidget::getCostValue(value);

  constexpr int presice = 3;
  QString text = "id, cost, safety, lateral, smoothness, consistency, garbage: \n";

  const auto &val_planning_trajectory = data.planning_trajectory;
  text += QString(
        "old %1,  %2,  %3,  %4,  %5,  %6,  %7").
      arg(val_planning_trajectory.id).
      arg(val_planning_trajectory.cost, 0, 'f', presice).
      arg(val_planning_trajectory.safety_cost, 0, 'f', presice).
      arg(val_planning_trajectory.lateral_cost, 0, 'f', presice).
      arg(val_planning_trajectory.smoothness_cost, 0, 'f', presice).
      arg(val_planning_trajectory.consistency_cost, 0, 'f', presice).
      arg(val_planning_trajectory.garbage_cost, 0, 'f', presice);
  text.append("\n");

  const auto &val_planning_trajectory_cost = data_cost.planning_trajectory;
  text += QString(
        "new %1,  %2,  %3,  %4,  %5,  %6,  %7").
      arg(val_planning_trajectory_cost.id).
      arg(val_planning_trajectory_cost.cost, 0, 'f', presice).
      arg(val_planning_trajectory_cost.safety_cost, 0, 'f', presice).
      arg(val_planning_trajectory_cost.lateral_cost, 0, 'f', presice).
      arg(val_planning_trajectory_cost.smoothness_cost, 0, 'f', presice).
      arg(val_planning_trajectory_cost.consistency_cost, 0, 'f', presice).
      arg(val_planning_trajectory_cost.garbage_cost, 0, 'f', presice);
  text.append("\n");

  text += QString(
        "new weight %1,  %2,  %3,  %4,  %5,  %6,  %7").
      arg("/").
      arg("/").
      arg(value[QPlanningCostWidget::Safety], 0, 'f', presice).
      arg(value[QPlanningCostWidget::Lateral], 0, 'f', presice).
      arg(value[QPlanningCostWidget::Smoothness], 0, 'f', presice).
      arg(value[QPlanningCostWidget::Consistency], 0, 'f', presice).
      arg(value[QPlanningCostWidget::Garbage], 0, 'f', presice);

  return text;
}
