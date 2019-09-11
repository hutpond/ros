/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDebugToolMainWnd.h
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 主界面
********************************************************/
#ifndef Q_DEBUG_TOOL_MAIN_WND_H
#define Q_DEBUG_TOOL_MAIN_WND_H

#include <QtWidgets/QMainWindow>

class QPlanningWidget;
class QTextBrowser;
class QDataDisplayWidget;
class QPlanningCostWidget;

class QDebugToolMainWnd : public QMainWindow
{
  Q_OBJECT

public:
  QDebugToolMainWnd(QWidget *parent = Q_NULLPTR);
  void stopProcess();

protected:
  void createMenu();
  void createPlanningToolBar();
  void createViewToolBar();
  void createSettingToolBar();
  void createHelpToolBar();

  void processPreShow(QWidget *);

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void showEvent(QShowEvent *);

protected slots:
  void onActionPlanningLiveDisplay();
  void onActionPlanningReplay();

  void onActionViewZoomIn();
  void onActionViewZoomOut();
  void onActionViewReset();
  void onActionReplaySpeed();

  void onActionShowTargets();
  void onActionHelpAbout();

private:
  QPlanningWidget *m_pWdgPlanning;

  QWidget *m_pWdgCurrent;

  QAction *m_pActionReplaySpeed;

  bool m_bFlagShowAllTargets;
  QAction *m_pActionShowTargets;

public:
  static QStatusBar *s_pStatusBar;
  static QTextBrowser *s_pTextBrowser;
  static QDataDisplayWidget *s_pDataDisplay;
  static QPlanningCostWidget *s_pWdgPlanningCost;
};

#endif  // Q_DEBUG_TOOL_MAIN_WND_H
