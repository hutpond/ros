/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDebugToolMainWnd.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 主界面
********************************************************/
#ifndef Q_DEBUG_TOOL_MAIN_WND_H
#define Q_DEBUG_TOOL_MAIN_WND_H

#include <QMainWindow>

class QStatusBar;
class QTextBrowser;
class QBaseWidget;
class QDataDisplayWidget;
class QPlanningCostWidget;
class QEditToolsWidget;
class QDecisionStateWidget;

class QDebugToolMainWnd : public QMainWindow
{
  Q_OBJECT

  enum
  {
    OldPlanning,
    NewPlanning,
    PlanningCount
  };

public:
  QDebugToolMainWnd(QWidget *parent = Q_NULLPTR);
  void stopProcess();

protected:
  void createMenu();
  void createPlanningToolBar();
  void createViewToolBar();
  void createSettingToolBar();
  void createHelpToolBar();

  void setWndTitle();

protected:
  virtual void resizeEvent(QResizeEvent *);

protected slots:
  void onActionOldPlanning();
  void onActionNewPlanning();

  void onActionPlanningLiveDisplay();
  void onActionPlanningReplay();

  void onActionTowDisplays();
  void onActionViewZoomIn();
  void onActionViewZoomOut();
  void onActionViewReset();
  void onActionReplaySpeed();

  void onActionChangeView();
  void onActionShowTargets();
  void onActionHelpAbout();

private:
  QBaseWidget *m_pWdgPlanning[PlanningCount];
  QEditToolsWidget *m_pWdgEditTool;

  int m_nCurrentIndex;

  QAction *m_pActionOldPlanning;
  QAction *m_pActionNewPlanning;
  QAction *m_pActionReplaySpeed;
  QAction *m_pActionShowTargets;
  QAction *m_pActionTowDisplays;

  bool m_bFlagShowAllTargets;

public:
  static QStatusBar *s_pStatusBar;
  static QTextBrowser *s_pTextBrowser;
  static QDataDisplayWidget *s_pDataDisplay;
  static QPlanningCostWidget *s_pWdgPlanningCost;
  static QDecisionStateWidget *s_pWdgDecisionState;
};

#endif  // Q_DEBUG_TOOL_MAIN_WND_H
