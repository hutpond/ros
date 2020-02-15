/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QPlanningParamWidget.h
 * Author: liuzheng
 * Date: 2019/6/21
 * Description: 路径规划参数子画面，显示相关参数和设定显示选项
********************************************************/
#ifndef Q_PLANNING_PARAM_WIDGET_H
#define Q_PLANNING_PARAM_WIDGET_H

#include <QWidget>
#include "QPlanningShowWidget.h"
#include "debug_tool/ads_PlanningData4Debug.h"

class QLabel;
class QGroupBox;
class QPushButton;
class QSlider;
class QTextBrowser;
class QCostValueWidget;
class QStateWidget;

class QPlanningParamWidget : public QWidget
{
  Q_OBJECT

  enum
  {
    DecisionUltraSonic,
    DecisionRadar28,
    DecisionRadar73,
    DecisionTrack,
    DecisionCount
  };

public:
  QPlanningParamWidget(QWidget *parent);
  ~QPlanningParamWidget();

  void setShowType(int);
  void setFrameCount(int);
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &,
                       const debug_tool::ads_PlanningData4Debug &);
  void showMousePosition(float, float, float, float);
  void setFrameOffset(int);

protected:
  virtual void resizeEvent(QResizeEvent *);

protected:
  void showReplayControls(bool);
  QString createTrajectoryString(const debug_tool::ads_PlanningData4Debug &,
                                 const debug_tool::ads_PlanningData4Debug &);

signals:
  void replayState(bool);
  void replayFrameOffset(int);
  void costValueChanged();

private slots:
  void onBtnClicked();
  void onSliderValueChanged(int);

private:
  QLabel *m_pLblMousePosName;
  QLabel *m_pLblMousePosValue;

  QStateWidget *m_pWdgState;
  QLabel *m_pLblDecisionValue[DecisionCount];

  QLabel *m_pLblScenarioType;

  QPushButton *m_pBtnPause;
  QPushButton *m_pBtnResume;
  QPushButton *m_pBtnBack;
  QPushButton *m_pBtnNext;
  QSlider *m_pSliderPlay;

  QCostValueWidget *m_pWdgCostValue;

  int m_nSliderValue;
};

#endif // Q_PLANNING_PARAM_WIDGET_H
