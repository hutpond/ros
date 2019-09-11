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

  m_pLblDecisionName = new QLabel(tr("Decision"), this);
  m_pLblDecisionValue = new QLabel(this);

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
    m_nSliderValue = 0;
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

void QPlanningParamWidget::setPlanningData(const debug_tool::ads_PlanningData4Debug &data)
{
  // 决策状态
  int nIndex = static_cast<int>(data.decision);
  int nIndexUs = static_cast<int>(data.ultrasonic_decision);
  int nIndexRadar = static_cast<int>(data.radar28f_decision);
  int nIndexRadar73 = static_cast<int>(data.radar73f_decision);
  int nIndexTrack = static_cast<int>(data.track_target_decision);
  m_pLblDecisionValue->setFont(G_TEXT_FONT);
  QString strText = QString("%1 [us: %2 r28: %3 r73: %4 track: %5]").
      arg(this->getDecisionText(nIndex)).
      arg(this->getDecisionText(nIndexUs)).
      arg(this->getDecisionText(nIndexRadar)).
      arg(this->getDecisionText(nIndexRadar73)).
      arg(this->getDecisionText(nIndexTrack));
  m_pLblDecisionValue->setText(strText);
  if (nIndex >= 0 && nIndex < 4) {
    m_pLblDecisionValue->setStyleSheet("background-color: rgb(0, 0, 0, 0);");
  }
  else {
    m_pLblDecisionValue->setStyleSheet("background-color: rgb(255, 0, 0);");
  }

  // data
  m_pTextBrowser->setText(this->createTrajectoryString(data));
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

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;

  m_pLblDecisionName->setGeometry(xPos, yPos, ITEM_NAME_W, ITEM_H);
  xPos += ITEM_NAME_W + SPACE_X;
  m_pLblDecisionValue->setGeometry(xPos, yPos, ITEM_VALUE_W, ITEM_H);
  yPos += ITEM_H + SPACE_Y;

  xPos = SPACE_X;
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

  xPos = SPACE_X;
  yPos += ITEM_H + SPACE_Y;
  m_pTextBrowser->setGeometry(xPos, yPos, WIDTH - 2 * SPACE_X, ITEM_H * 20);
}

void QPlanningParamWidget::showReplayControls(bool show)
{
  m_pBtnPause->setHidden(!show);
  m_pBtnResume->setHidden(!show);
  m_pBtnBack->setHidden(!show);
  m_pBtnNext->setHidden(!show);
  m_pSliderPlay->setHidden(!show);
}

QString QPlanningParamWidget::createTrajectoryString(const debug_tool::ads_PlanningData4Debug &data)
{
  constexpr int presice = 3;
  QString text = "id, cost, safety, lateral, smoothness, consistency, garbage: \n";

  auto val_candidates = data.planning_trajectory_candidates;
  int size_candidates = val_candidates.size();
  if (size_candidates > 10) {
    using type_candidates = decltype(val_candidates[0]);
    std::sort(val_candidates.begin(), val_candidates.end(), [](const type_candidates &val,
              const type_candidates &val2){
      return val.cost < val2.cost;
    });
  }

  size_candidates = qBound<int>(0, size_candidates, 10);
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
