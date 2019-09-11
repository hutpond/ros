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

class QPlanningParamWidget : public QWidget
{
  Q_OBJECT

public:
  QPlanningParamWidget(QWidget *parent);
  ~QPlanningParamWidget();

  void setShowType(int);
  void setFrameCount(int);
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);
  void showMousePosition(float, float, float, float);
  void setFrameOffset(int);

protected:
  virtual void resizeEvent(QResizeEvent *);

protected:
  QString getDecisionText(int);
  void showReplayControls(bool);
  QString createTrajectoryString(const debug_tool::ads_PlanningData4Debug &);

signals:
  void replayState(bool);
signals:
  void replayFrameOffset(int);

private slots:
  void onBtnClicked();
  void onSliderValueChanged(int);

private:
  QLabel *m_pLblMousePosName;
  QLabel *m_pLblMousePosValue;

  QLabel *m_pLblDecisionName;
  QLabel *m_pLblDecisionValue;

  QPushButton *m_pBtnPause;
  QPushButton *m_pBtnResume;
  QPushButton *m_pBtnBack;
  QPushButton *m_pBtnNext;
  QSlider *m_pSliderPlay;

  QTextBrowser *m_pTextBrowser;

  int m_nSliderValue;
};

#endif // Q_PLANNING_PARAM_WIDGET_H
