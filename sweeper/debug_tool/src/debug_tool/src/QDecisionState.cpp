#include "QDecisionState.h"

#include <QLabel>
#include "qstateitem.h"

QDecisionState::QDecisionState(QWidget *parent) : QWidget(parent)
{
  for (int i = 0; i < 10; ++i) {
    m_pLblName[i] = new QLabel(this);
    m_pLblDecision[i] = new QLabel(this);
    m_pLblDecision[i]->setText("");
  }
  m_pLblName[0]->setText("overall");
  m_pLblName[1]->setText("preprocess");
  m_pLblName[2]->setText("safe_stop");
  m_pLblName[3]->setText("traffic_light");
  m_pLblName[4]->setText("radar");
  m_pLblName[5]->setText("ultrasonic");
  m_pLblName[6]->setText("lidar");
  m_pLblName[7]->setText("front_target");
  m_pLblName[8]->setText("route");
  m_pLblName[9]->setText("rear_target");
}

void QDecisionState::setData(const debug_tool::ads_PlanningData4Debug &data)
{
  m_pLblDecision[0]->setText(getDecisionText(data.overall_decision));
  m_pLblDecision[1]->setText(getDecisionText(data.preprocess_decision));
  m_pLblDecision[2]->setText(getDecisionText(data.safe_stop_decision));
  m_pLblDecision[3]->setText(getDecisionText(data.traffic_light_decision));
  m_pLblDecision[4]->setText(getDecisionText(data.radar_decision));
  m_pLblDecision[5]->setText(getDecisionText(data.ultrasonic_decision));
  m_pLblDecision[6]->setText(getDecisionText(data.lidar_target_decision));
  m_pLblDecision[7]->setText(getDecisionText(data.front_target_decision));
  m_pLblDecision[8]->setText(getDecisionText(data.route_decision));
  m_pLblDecision[9]->setText(getDecisionText(data.rear_target_decision));
}

QString QDecisionState::getDecisionText(uint8_t decision)
{
  QString text;
  switch (decision) {
    case debug_tool::ads_PlanningData4Debug::Forward:
      text = QStringLiteral("前进");
      break;
    case debug_tool::ads_PlanningData4Debug::LeftPass:
      text = QStringLiteral("左侧绕行");
      break;
    case debug_tool::ads_PlanningData4Debug::RightPass:
      text = QStringLiteral("右侧绕行");
      break;
    case debug_tool::ads_PlanningData4Debug::Follow:
      text = QStringLiteral("跟随");
      break;
    case debug_tool::ads_PlanningData4Debug::Wait:
      text = QStringLiteral("停车等待");
      break;
    case debug_tool::ads_PlanningData4Debug::SafeStop:
      text = QStringLiteral("安全停车");
      break;
    case debug_tool::ads_PlanningData4Debug::Exit:
      text = QStringLiteral("退出");
      break;
    default:
      break;
  }
  return text;
}

void QDecisionState::resizeEvent(QResizeEvent *)
{
    const int WIDTH = this->width();
    const int HEIGHT = this->height();

    const float ITEM_H = float(HEIGHT) / 10;
    const float NAME_W_PF = 0.4f;

    for (int i = 0; i < 10; ++i) {
      m_pLblName[i]->setGeometry(0, i * ITEM_H, WIDTH * NAME_W_PF, ITEM_H);
      m_pLblDecision[i]->setGeometry(
            WIDTH * NAME_W_PF, i * ITEM_H, WIDTH * (1.0f - NAME_W_PF), ITEM_H);
    }
}
