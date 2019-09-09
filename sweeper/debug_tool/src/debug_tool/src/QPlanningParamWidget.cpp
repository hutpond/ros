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
#include <QTextBrowser>
#include "QPlanningParamWidget.h"
#include "QPlanningWidget.h"
#include "GlobalDefine.h"

QPlanningParamWidget::QPlanningParamWidget(QWidget *parent)
  : QWidget(parent)
{
  m_pLblMousePosName = new QLabel(tr("鼠标坐标"), this);
  m_pLblMousePosValue = new QLabel(this);

  for (int i = 0; i < 2; ++i) {
    m_pLblDecisionName[i] = new QLabel(tr("Decision"), this);
    m_pLblDecisionValue[i] = new QLabel(this);

    m_pBtnPause[i] = new QPushButton(tr("暂停"), this);
    connect(m_pBtnPause[i], &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

    m_pBtnResume[i] = new QPushButton(tr("恢复"), this);
    connect(m_pBtnResume[i], &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

    m_pBtnBack[i] = new QPushButton(tr("后退"), this);
    connect(m_pBtnBack[i], &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

    m_pBtnNext[i] = new QPushButton(tr("前进"), this);
    connect(m_pBtnNext[i], &QPushButton::clicked, this, &QPlanningParamWidget::onBtnClicked);

    m_pSliderPlay[i] = new QSlider(Qt::Horizontal, this);
    connect(m_pSliderPlay[i], &QSlider::valueChanged,
            this, &QPlanningParamWidget::onSliderValueChanged);
  }

  m_pTextBrowser = new QTextBrowser(this);
}

QPlanningParamWidget::~QPlanningParamWidget()
{
}

/*******************************************************
 * @brief 设置显示类型：Live Display & Replay
 * @param

 * @return
********************************************************/
void QPlanningParamWidget::setShowType(int index)
{
  m_pBtnPause[index]->setEnabled(true);
  m_pBtnResume[index]->setEnabled(false);
  m_pBtnBack[index]->setEnabled(false);
  m_pBtnNext[index]->setEnabled(false);
  m_pSliderPlay[index]->setEnabled(false);
  m_nSliderValue[index] = 0;
}

/*******************************************************
 * @brief 设置数据总帧数，用于设置拖动条范围
 * @param count: 数据总帧数

 * @return
********************************************************/
void QPlanningParamWidget::setFrameCount(int index, int count)
{
  m_pSliderPlay[index]->setMinimum(0);
  m_pSliderPlay[index]->setMaximum(count);
  m_pSliderPlay[index]->setPageStep(1);
  m_pSliderPlay[index]->setSingleStep(1);

  m_pSliderPlay[index]->setValue(0);
  m_nSliderValue[index] = 0;
}

void QPlanningParamWidget::setPlanningData(int i, bool cost, const debug_tool::ads_PlanningData4Debug &data)
{
  // 决策状态
  int nIndex = static_cast<int>(data.decision);
  int nIndexUs = static_cast<int>(data.ultrasonic_decision);
  int nIndexRadar = static_cast<int>(data.radar28f_decision);
  int nIndexRadar73 = static_cast<int>(data.radar73f_decision);
  int nIndexTrack = static_cast<int>(data.track_target_decision);
  m_pLblDecisionValue[i]->setFont(G_TEXT_FONT);
  QString strText = QString("%1 [us: %2 r28: %3 r73: %4 track: %5]").
      arg(this->getDecisionText(nIndex)).
      arg(this->getDecisionText(nIndexUs)).
      arg(this->getDecisionText(nIndexRadar)).
      arg(this->getDecisionText(nIndexRadar73)).
      arg(this->getDecisionText(nIndexTrack));
  m_pLblDecisionValue[i]->setText(strText);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue[i]->setStyleSheet("background-color: rgb(0, 0, 0, 0);");
  }
  else {
    m_pLblDecisionValue[i]->setStyleSheet("background-color: rgb(255, 0, 0);");
  }

  // data
  if (cost) {
    m_pTextBrowser->setText(this->createTrajectoryString(data));
  }
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
void QPlanningParamWidget::setFrameOffset(int index, int offset)
{
  int nValue = m_nSliderValue[index] + offset;
  m_nSliderValue[index] = qBound<int>(m_pSliderPlay[index]->minimum(),
                                      nValue, m_pSliderPlay[index]->maximum());
  m_pSliderPlay[index]->setValue(m_nSliderValue[index]);
}

/*******************************************************
 * @brief 回放button点击响应槽函数
 * @param

 * @return
********************************************************/
void QPlanningParamWidget::onBtnClicked()
{
  QObject *sender = this->sender();
  if (sender == m_pBtnPause[0]) {
    m_pBtnPause[0]->setEnabled(false);
    m_pBtnResume[0]->setEnabled(true);
    m_pBtnBack[0]->setEnabled(true);
    m_pBtnNext[0]->setEnabled(true);
    m_pSliderPlay[0]->setEnabled(true);
    emit replayState(0, true);
  }
  else if (sender == m_pBtnPause[1]) {
    m_pBtnPause[1]->setEnabled(false);
    m_pBtnResume[1]->setEnabled(true);
    m_pBtnBack[1]->setEnabled(true);
    m_pBtnNext[1]->setEnabled(true);
    m_pSliderPlay[1]->setEnabled(true);
    emit replayState(1, true);
  }
  else if (sender == m_pBtnResume[0]) {
    m_pBtnPause[0]->setEnabled(true);
    m_pBtnResume[0]->setEnabled(false);
    m_pBtnBack[0]->setEnabled(false);
    m_pBtnNext[0]->setEnabled(false);
    m_pSliderPlay[0]->setEnabled(false);
    emit replayState(0, false);
  }
  else if (sender == m_pBtnResume[1]) {
    m_pBtnPause[1]->setEnabled(true);
    m_pBtnResume[1]->setEnabled(false);
    m_pBtnBack[1]->setEnabled(false);
    m_pBtnNext[1]->setEnabled(false);
    m_pSliderPlay[1]->setEnabled(false);
    emit replayState(1, false);
  }
  else if (sender == m_pBtnBack[0]) {
    this->setFrameOffset(0, -1);
    emit replayFrameOffset(0, -1);
  }
  else if (sender == m_pBtnBack[1]) {
    this->setFrameOffset(1, -1);
    emit replayFrameOffset(1, -1);
  }
  else if (sender == m_pBtnNext[0]) {
    this->setFrameOffset(0, 1);
    emit replayFrameOffset(0, 1);
  }
  else if (sender == m_pBtnNext[1]) {
    this->setFrameOffset(1, 1);
    emit replayFrameOffset(1, 1);
  }
}

/*******************************************************
 * @brief slider值改变响应槽函数
 * @param value: slider改变后的值

 * @return
********************************************************/
void QPlanningParamWidget::onSliderValueChanged(int value)
{
  QObject *sender = this->sender();
  for (int i = 0; i < 2; ++i) {
    if (sender == m_pSliderPlay[i] && m_nSliderValue[i] != value) {
      emit replayFrameOffset(i, value - m_nSliderValue[i]);
      m_nSliderValue[i] = value;
    }
  }
}

void QPlanningParamWidget::resizeEvent(QResizeEvent *)
{
  const float F_SPACE_X_P = 0.01f;
  const float F_SPACE_Y_P = 0.02f;
  const float F_ITEM_H_P = 0.037f;
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
  m_pLblMousePosName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblMousePosValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);

  const int size_btn = 4;
  const int BTN_W = (WIDTH - SPACE_X * (size_btn + 1))  / size_btn;

  for (int i = 0; i < 2; ++i) {
    xPos = SPACE_X;
    yPos += ITEM_H + SPACE_Y;

    m_pLblDecisionName[i]->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
    xPos += ITEM_NAME_W + SPACE_X;
    m_pLblDecisionValue[i]->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);
    yPos += ITEM_H + SPACE_Y;

    xPos = SPACE_X;
    m_pBtnPause[i]->setGeometry(xPos, yPos, BTN_W, ITEM_H);
    xPos += BTN_W + SPACE_X;
    m_pBtnResume[i]->setGeometry(xPos, yPos, BTN_W, ITEM_H);
    xPos += BTN_W + SPACE_X;
    m_pBtnBack[i]->setGeometry(xPos, yPos, BTN_W, ITEM_H);
    xPos += BTN_W + SPACE_X;
    m_pBtnNext[i]->setGeometry(xPos, yPos, BTN_W, ITEM_H);

    xPos = SPACE_X;
    yPos += ITEM_H + SPACE_Y;
    m_pSliderPlay[i]->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H);
  }

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pTextBrowser->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H * 20);
}

void QPlanningParamWidget::showReplayControls(bool show)
{
  for (int i = 0; i < 2; ++i) {
    m_pBtnPause[i]->setHidden(!show);
    m_pBtnResume[i]->setHidden(!show);
    m_pBtnBack[i]->setHidden(!show);
    m_pBtnNext[i]->setHidden(!show);
    m_pSliderPlay[i]->setHidden(!show);
  }
}

QString QPlanningParamWidget::createTrajectoryString(const debug_tool::ads_PlanningData4Debug &data)
{
  constexpr int presice = 3;
  QString text = "id, cost, safety, lateral, smoothness, consistency, garbage: \n";

  const auto &val_candidates = data.planning_trajectory_candidates;
  const int size_candidates = qBound<int>(0, val_candidates.size(), 10);
  for (int i = 0; i < size_candidates; ++ i) {
    text += QString(
          "%1, %2, %3, %4, %5, %6, %7").
        arg(val_candidates[i].id).
        arg(val_candidates[i].cost, 0, 'f', presice).
        arg(val_candidates[i].safety_cost, 0, 'f', presice).
        arg(val_candidates[i].lateral_cost, 0, 'f', presice).
        arg(val_candidates[i].smoothness_cost, 0, 'f', presice).
        arg(val_candidates[i].consistency_cost, 0, 'f', presice).
        arg(val_candidates[i].garbage_cost, 0, 'f', presice);
    text.append('\n');
  }

  const auto &val_planning_trajectory = data.planning_trajectory;
  text += QString(
        "* %1, %2, %3, %4, %5, %6, %7").
      arg(val_planning_trajectory.id).
      arg(val_planning_trajectory.cost, 0, 'f', presice).
      arg(val_planning_trajectory.safety_cost, 0, 'f', presice).
      arg(val_planning_trajectory.lateral_cost, 0, 'f', presice).
      arg(val_planning_trajectory.smoothness_cost, 0, 'f', presice).
      arg(val_planning_trajectory.consistency_cost, 0, 'f', presice).
      arg(val_planning_trajectory.garbage_cost, 0, 'f', presice);

  return text;
}
