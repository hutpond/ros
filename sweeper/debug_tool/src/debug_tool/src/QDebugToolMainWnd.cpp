/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDebugToolMainWnd.cpp
 * Author: liuzheng
 * Date: 2019/6/19
 * Description: 主界面
********************************************************/
#include <QToolBar>
#include <QStatusBar>
#include <QMenu>
#include <QMenuBar>
#include <QFileDialog>
#include <QTextBrowser>
#include <QDockWidget>
#include "QDebugToolMainWnd.h"
#include "QPlanningWidget.h"
#include "QDataDisplayWidget.h"
#include "QPlanningCostWidget.h"
#include "QEditToolsWidget.h"
#include "QNewPlanningWidget.h"

static const int TOOL_BAR_ACTION_SIZE = 40;
static const char *PLANNING_TOOL_BAR = "PLANNING";
static const char *VIEW_TOOL_BAR = "VIEW";
static const char *SETTINT_TOOL_BAR = "SETTING";
static const char *HELP_TOOL_BAR = "HELP";

static const char *WND_TITLE = "Debug Tool V3.4.6";

QStatusBar * QDebugToolMainWnd::s_pStatusBar = Q_NULLPTR;
QTextBrowser * QDebugToolMainWnd::s_pTextBrowser = Q_NULLPTR;
QDataDisplayWidget * QDebugToolMainWnd::s_pDataDisplay = Q_NULLPTR;
QPlanningCostWidget * QDebugToolMainWnd::s_pWdgPlanningCost = Q_NULLPTR;

QDebugToolMainWnd::QDebugToolMainWnd(QWidget *parent)
  : QMainWindow(parent)
  , m_pWdgCurrent(Q_NULLPTR)
{
  m_pWdgEditTool = new QEditToolsWidget(this);
  QDockWidget *pDockWdg = new QDockWidget("", this);
  pDockWdg->setFeatures(QDockWidget::AllDockWidgetFeatures);
  this->addDockWidget(Qt::LeftDockWidgetArea, pDockWdg);
  pDockWdg->setWidget(m_pWdgEditTool);

  this->setCentralWidget(new QWidget);

  m_pWdgPlanning[Planning] = new QPlanningWidget(this->centralWidget());
  connect(m_pWdgEditTool, &QEditToolsWidget::selectTool,
          m_pWdgPlanning[Planning], &QPlanningWidget::onSelectTool);
  m_pWdgPlanning[Planning]->onSelectTool(QEditToolsWidget::Move, true);
  m_pWdgCurrent = m_pWdgPlanning[Planning];

  m_pWdgPlanning[NewPlanning] = new QNewPlanningWidget(this->centralWidget());
  connect(m_pWdgEditTool, &QEditToolsWidget::selectTool,
          m_pWdgPlanning[NewPlanning], &QPlanningWidget::onSelectTool);
  m_pWdgPlanning[NewPlanning]->onSelectTool(QEditToolsWidget::Move, true);
  m_pWdgPlanning[NewPlanning]->hide();

  this->createMenu();
  this->createPlanningToolBar();
  this->createViewToolBar();
  this->createSettingToolBar();

  this->setWndTitle();

  pDockWdg = new QDockWidget("", this);
  pDockWdg->setFeatures(QDockWidget::AllDockWidgetFeatures);
  QWidget *pDockTitle = new QWidget;
  pDockTitle->setStyleSheet("background-color: rgb(114, 159, 207);");
  pDockWdg->setTitleBarWidget(pDockTitle);
  this->addDockWidget(Qt::BottomDockWidgetArea, pDockWdg);

  s_pTextBrowser = new QTextBrowser(this);
  QTabWidget *pTabWidget = new QTabWidget(this);
  pTabWidget->addTab(s_pTextBrowser, tr("Output"));
  s_pDataDisplay = new QDataDisplayWidget(this);
  pTabWidget->addTab(s_pDataDisplay, tr("Data"));

  // plannig cost
  s_pWdgPlanningCost = new QPlanningCostWidget(this);
  pTabWidget->addTab(s_pWdgPlanningCost, tr("Cost"));

  pDockWdg->setWidget(pTabWidget);

  s_pStatusBar = this->statusBar();
  s_pStatusBar->showMessage("Ready");
}

/*******************************************************
 * @brief 停止相关资源、线程
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::stopProcess()
{
  m_pWdgPlanning[Planning]->stopDisplay();
  m_pWdgPlanning[NewPlanning]->stopDisplay();
}

void QDebugToolMainWnd::resizeEvent(QResizeEvent *)
{
  QRect rect = this->centralWidget()->rect();
  m_pWdgPlanning[Planning]->setGeometry(rect);
  m_pWdgPlanning[NewPlanning]->setGeometry(rect);
}

/*******************************************************
 * @brief 创建菜单
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createMenu()
{
  QMenu *menu = menuBar()->addMenu(tr("Planning"));
  menu->addAction(tr("Live Display"), this,
                  SLOT(onActionPlanningLiveDisplay()));
  menu->addAction(tr("Replay"), this, SLOT(onActionPlanningReplay()));

  menu = menuBar()->addMenu(tr("View"));
  menu->addAction(tr("Zoom In +"), this, SLOT(onActionViewZoomIn()));
  menu->addAction(tr("Zoom Out -"), this, SLOT(onActionViewZoomOut()));
  menu->addAction(tr("Reset R"), this, SLOT(onActionViewReset()));

  menu = menuBar()->addMenu(tr("Help"));
  menu->addAction(tr("About Debug Tool"), this,
                  SLOT(onActionHelpAbout()));
}

/*******************************************************
 * @brief 创建Planning模块对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createPlanningToolBar()
{
  QToolBar *pToolBar = this->addToolBar(PLANNING_TOOL_BAR);

  QAction *newAct = new QAction(tr("SWITCH"), this);
  newAct->setToolTip("switch live and replay");
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPlanningLiveDisplay);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 250, 220);");

  newAct = new QAction(tr("REPLAY"), this);
  newAct->setToolTip("open replay path");
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPlanningReplay);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 250, 220);");
  pToolBar->insertSeparator(newAct);
}

/*******************************************************
 * @brief 实时显示路径规划相关的当前数据、状态
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionPlanningLiveDisplay()
{
  int type = m_pWdgPlanning[Planning]->showType();
  type = (type == QPlanningWidget::LivePlay ?
            QPlanningWidget::RePlay : QPlanningWidget::LivePlay);
  m_pWdgPlanning[Planning]->setShowType(type);
  m_pWdgPlanning[NewPlanning]->setShowType(type);
  this->setWndTitle();
}

/*******************************************************
 * @brief 回放路径规划相关的当前数据、状态
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionPlanningReplay()
{
  static QString strOpenPath;
  if (strOpenPath.isEmpty()) {
    namespace fs = boost::filesystem;
    fs::path fsPath = getenv("HOME");
    fsPath /= "PlanningData";
    strOpenPath = QString::fromStdString(fsPath.string());
  }
  QString strPath = QFileDialog::getExistingDirectory(this, tr("Json Directory"),
                                    strOpenPath,
                                    QFileDialog::ShowDirsOnly
                                    | QFileDialog::DontResolveSymlinks);
  if (!strPath.isEmpty()) {
    int index = strPath.lastIndexOf('/');
    strOpenPath = strPath.mid(0, index);
    m_pWdgCurrent->startReplay(strPath);
    m_pWdgPlanning[Planning]->setShowType(QPlanningWidget::RePlay);
    m_pWdgPlanning[NewPlanning]->setShowType(QPlanningWidget::RePlay);
    this->setWndTitle();
  }
}

/*******************************************************
 * @brief 创建View对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createViewToolBar()
{
  QToolBar *pToolBar = this->addToolBar(VIEW_TOOL_BAR);

  QAction *newAct = new QAction(tr("+"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionViewZoomIn);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 200, 200);");

  newAct = new QAction(tr("-"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionViewZoomOut);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 200, 200);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("R"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionViewReset);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 200, 200);");
  pToolBar->insertSeparator(newAct);

  m_pActionReplaySpeed = new QAction(tr("M"), this);
  connect(m_pActionReplaySpeed, &QAction::triggered,
          this, &QDebugToolMainWnd::onActionReplaySpeed);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionReplaySpeed);
  pWdgAction = pToolBar->widgetForAction(m_pActionReplaySpeed);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 200, 200);");
  pToolBar->insertSeparator(m_pActionReplaySpeed);
}

/*******************************************************
 * @brief 放大显示当前调试画面，如果当前画面无放大功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewZoomIn()
{
  m_pWdgCurrent->setViewResolution(1);
}

/*******************************************************
 * @brief 缩小显示当前调试画面，如果当前画面无缩小功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewZoomOut()
{
  m_pWdgCurrent->setViewResolution(-1);
}

/*******************************************************
 * @brief 复位当前显示画面到原始显示大小，如果当前画面无放大、缩小功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewReset()
{
  m_pWdgCurrent->setViewResolution(0);
}

/*******************************************************
 * @brief 创建Setting对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createSettingToolBar()
{
  QToolBar *pToolBar = this->addToolBar(SETTINT_TOOL_BAR);

  QAction *action = new QAction(tr("=>>"), this);
  connect(action, &QAction::triggered,
          this, &QDebugToolMainWnd::onActionChangeView);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(action);
  QWidget *pWdgAction = pToolBar->widgetForAction(action);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(200, 255, 200);");
  pToolBar->insertSeparator(action);

  m_bFlagShowAllTargets = false;
  m_pActionShowTargets = new QAction(tr("T"), this);
  connect(m_pActionShowTargets, &QAction::triggered,
          this, &QDebugToolMainWnd::onActionShowTargets);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionShowTargets);
  pWdgAction = pToolBar->widgetForAction(m_pActionShowTargets);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(200, 255, 200);");
  pToolBar->insertSeparator(m_pActionShowTargets);
}

void QDebugToolMainWnd::onActionChangeView()
{
  m_pWdgPlanning[Planning]->changeShowView();
  m_pWdgPlanning[NewPlanning]->changeShowView();
  this->setWndTitle();
}

/*******************************************************
 * @brief 设置是否全部显示target
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionShowTargets()
{
  m_bFlagShowAllTargets = !m_bFlagShowAllTargets;
  if (m_bFlagShowAllTargets) {
    m_pActionShowTargets->setText(tr("T A"));
  }
  else {
    m_pActionShowTargets->setText(tr("T"));
  }
  m_pActionShowTargets->setChecked(m_bFlagShowAllTargets);
  m_pWdgPlanning[Planning]->setShowAllTargets(m_bFlagShowAllTargets);
  m_pWdgPlanning[NewPlanning]->setShowAllTargets(m_bFlagShowAllTargets);
}

void QDebugToolMainWnd::onActionReplaySpeed()
{
  int index = m_pWdgPlanning[Planning]->replaySpeedIndex();
  ++ index %= 3;
  m_pWdgPlanning[Planning]->setReplaySpeedIndex(index);
  m_pWdgPlanning[NewPlanning]->setReplaySpeedIndex(index);
  if (index == 0) {
    m_pActionReplaySpeed->setText(tr("H"));
  }
  else if (index == 1) {
    m_pActionReplaySpeed->setText(tr("M"));
  }
  else {
    m_pActionReplaySpeed->setText(tr("L"));
  }
}

/*******************************************************
 * @brief 创建Help对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createHelpToolBar()
{
  QToolBar *pToolBar = this->addToolBar(HELP_TOOL_BAR);

  QAction *newAct = new QAction(tr("ABOUT"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionHelpAbout);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(200, 200, 255);");
}

/*******************************************************
 * @brief 显示Debug Tool相关信息
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionHelpAbout()
{

}

void QDebugToolMainWnd::setWndTitle()
{
  int type = m_pWdgPlanning[Planning]->showType();
  int view = m_pWdgPlanning[Planning]->showView();
  QString title = WND_TITLE;
  title.append(" - ");
  title.append(type == QPlanningWidget::LivePlay ? "LIVE" : "REPLAY");
  title.append(" - ");
  title.append(view == QPlanningWidget::LocalView ? "LOCAL" : "FULL");
  this->setWindowTitle(title);
}
