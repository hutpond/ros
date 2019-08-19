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

class QLocationWidget;
class QVarianceWidget;
class QAlgorithmWidget;
class QLocusWidget;
class QEulerAngleWidget;
class QErrorFigureWidget;
class QPerceptionWidget;
class QPlanningWidget;
class QTextBrowser;
class QDataDisplayDialog;

class QDebugToolMainWnd : public QMainWindow
{
  Q_OBJECT

public:
  QDebugToolMainWnd(QWidget *parent = Q_NULLPTR);
  void stopProcess();

protected:
  void createMenu();
  void createLocationToolBar();
  void createPerceptionToolBar();
  void createPlanningToolBar();
  void createViewToolBar();
  void createSettingToolBar();
  void createHelpToolBar();

  void processPreShow(QWidget *);

protected:
  virtual void resizeEvent(QResizeEvent *);
  virtual void showEvent(QShowEvent *);

protected slots:
  void onActionLocation();
  void onActionVariance();
  void onActionAlgorithm();
  void onActionLocus();
  void onActionEulerAngle();
  void onActionErrorFigure();

  void onActionPerception2D();
  void onActionPerception3D();
  void onActionPerceptionReplay();

  void onActionPlanningLiveDisplay();
  void onActionPlanningReplay();

  void onActionViewZoomIn();
  void onActionViewZoomOut();
  void onActionViewReset();

  void onActionSettingSaveJson();
  void onActionHelpAbout();

private:
  QLocationWidget *m_pWdgLocation;
  QVarianceWidget *m_pWdgVariance;
  QAlgorithmWidget *m_pWdgAlgorithm;
  QLocusWidget *m_pWdgLocus;
  QEulerAngleWidget *m_pWdgEulerAngle;
  QErrorFigureWidget *m_pWdgErrorFigure;

  QPerceptionWidget *m_pWdgPerception;
  QPlanningWidget *m_pWdgPlanning;

  QWidget *m_pWdgCurrent;

  bool m_bFlagSaveJson;
  QAction *m_pActionSaveJson;
  QAction *m_pActionMenuSaveJson;

public:
  static QStatusBar *s_pStatusBar;
  static QTextBrowser *s_pTextBrowser;
  static QDataDisplayDialog *s_pDataDisplay;
};

#endif  // Q_DEBUG_TOOL_MAIN_WND_H
