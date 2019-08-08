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
#include "QLocationWidget.h"
#include "QVarianceWidget.h"
#include "QAlgorithmWidget.h"
#include "QLocusWidget.h"
#include "QEulerAngleWidget.h"
#include "QErrorFigureWidget.h"
#include "QPerceptionWidget.h"
#include "QPlanningWidget.h"
//#include "ReadDataManager.h"

static const int TOOL_BAR_ACTION_SIZE = 40;
static const char *LOCATION_TOOL_BAR = "LOCATION";
static const char *PERCEPTION_TOOL_BAR = "PERCEPTION";
static const char *PLANNING_TOOL_BAR = "PLANNING";
static const char *VIEW_TOOL_BAR = "VIEW";
static const char *SETTINT_TOOL_BAR = "SETTING";
static const char *HELP_TOOL_BAR = "HELP";

QStatusBar * QDebugToolMainWnd::s_pStatusBar = NULL;
QTextBrowser * QDebugToolMainWnd::s_pTextBrowser = NULL;

QDebugToolMainWnd::QDebugToolMainWnd(QWidget *parent)
  : QMainWindow(parent)
  , m_pWdgCurrent(Q_NULLPTR)
{
  this->setCentralWidget(new QWidget);
  m_pWdgLocation = new QLocationWidget(this->centralWidget());
  m_pWdgLocation->hide();
  m_pWdgVariance = new QVarianceWidget(this->centralWidget());
  m_pWdgVariance->hide();
  m_pWdgAlgorithm = new QAlgorithmWidget(this->centralWidget());
  m_pWdgAlgorithm->hide();
  m_pWdgLocus = new QLocusWidget(this->centralWidget());
  m_pWdgLocus->hide();
  m_pWdgEulerAngle = new QEulerAngleWidget(this->centralWidget());
  m_pWdgEulerAngle->hide();
  m_pWdgErrorFigure = new QErrorFigureWidget(this->centralWidget());
  m_pWdgErrorFigure->hide();

  m_pWdgPerception = new QPerceptionWidget(this->centralWidget());
  m_pWdgPerception->hide();
  m_pWdgPlanning = new QPlanningWidget(this->centralWidget());
  m_pWdgPlanning->hide();

  this->createMenu();
  //this->createLocationToolBar();
  this->createPerceptionToolBar();
  this->createPlanningToolBar();
  this->createViewToolBar();
  //this->createSettingToolBar();
  //this->createHelpToolBar();

  this->setWindowTitle("Debug Tool V2.4");

  // set zmq callback function
  //auto funPlanning = std::bind(&QPlanningWidget::parseJsonData, m_pWdgPlanning,
  //	std::placeholders::_1);
  //ReadDataManager::instance()->setPlanningParseFunction(funPlanning);

  s_pTextBrowser = new QTextBrowser(this);
  QDockWidget *pDockWdg = new QDockWidget(tr("Output"), this);
  pDockWdg->setFeatures(QDockWidget::AllDockWidgetFeatures);
  pDockWdg->setWidget(s_pTextBrowser);
  QWidget *pDockTitle = new QWidget;
  pDockTitle->setStyleSheet("background-color: rgb(114, 159, 207);");
  pDockWdg->setTitleBarWidget(pDockTitle);
  this->addDockWidget(Qt::BottomDockWidgetArea, pDockWdg);

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
  m_pWdgPerception->stopDisplay();
  m_pWdgPlanning->stopDisplay();
}

void QDebugToolMainWnd::resizeEvent(QResizeEvent *)
{
  QRect rect = this->centralWidget()->rect();
  m_pWdgLocation->setGeometry(rect);
  m_pWdgVariance->setGeometry(rect);
  m_pWdgAlgorithm->setGeometry(rect);
  m_pWdgLocus->setGeometry(rect);
  m_pWdgEulerAngle->setGeometry(rect);
  m_pWdgErrorFigure->setGeometry(rect);
  m_pWdgPerception->setGeometry(rect);
  m_pWdgPlanning->setGeometry(rect);
}

void QDebugToolMainWnd::showEvent(QShowEvent *)
{
  if (m_pWdgCurrent == NULL) {
    this->onActionPlanningLiveDisplay();
  }
}

/*******************************************************
 * @brief 创建菜单
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createMenu()
{
  QMenu *menu = menuBar()->addMenu(tr("融合定位"));
  menu->addAction(tr("定位模块"), this, SLOT(onActionLocation()));
  menu->addAction(tr("信息方差"), this, SLOT(onActionVariance()));
  menu->addAction(tr("算法比较"), this, SLOT(onActionAlgorithm()));
  menu->addAction(tr("位置轨迹"), this, SLOT(onActionLocus()));
  menu->addAction(tr("姿态角"), this, SLOT(onActionEulerAngle()));
  menu->addAction(tr("Error"), this, SLOT(onActionErrorFigure()));

  menu = menuBar()->addMenu(tr("Perception"));
  menu->addAction(tr("2D"), this, SLOT(onActionPerception2D()));
  menu->addAction(tr("3D"), this, SLOT(onActionPerception3D()));
  menu->addAction(tr("Replay"), this, SLOT(onActionPerceptionReplay()));

  menu = menuBar()->addMenu(tr("Planning"));
  menu->addAction(tr("Live Display"), this,
                  SLOT(onActionPlanningLiveDisplay()));
  menu->addAction(tr("Replay"), this, SLOT(onActionPlanningReplay()));

  menu = menuBar()->addMenu(tr("View"));
  menu->addAction(tr("Zoom In +"), this, SLOT(onActionViewZoomIn()));
  menu->addAction(tr("Zoom Out -"), this, SLOT(onActionViewZoomOut()));
  menu->addAction(tr("Reset R"), this, SLOT(onActionViewReset()));

//  menu = menuBar()->addMenu(tr("Setting"));
//  m_pActionMenuSaveJson = menu->addAction(tr("Save Json"),
//                                          this, SLOT(onActionSettingSaveJson));
//  m_pActionMenuSaveJson->setCheckable(true);
//  m_pActionMenuSaveJson->setChecked(false);

  menu = menuBar()->addMenu(tr("Help"));
  menu->addAction(tr("About Debug Tool"), this,
                  SLOT(onActionHelpAbout()));
}

/*******************************************************
 * @brief hide前画面，显示w，并且将w赋值给当前画面指针
 * @param w: 前一画面指针

 * @return
********************************************************/
void QDebugToolMainWnd::processPreShow(QWidget *w)
{
  if (m_pWdgCurrent != w) {
    if (m_pWdgCurrent) {
      m_pWdgCurrent->hide();
    }
    m_pWdgCurrent = w;
  }
}

/*******************************************************
 * @brief 创建融合定位对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createLocationToolBar()
{
  QToolBar *pToolBar = this->addToolBar(LOCATION_TOOL_BAR);

  QAction *newAct = new QAction(tr("定位\n模块"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionLocation()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");

  newAct = new QAction(tr("信息\n方差"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionVariance()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("算法\n比较"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionAlgorithm()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("位置\n轨迹"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionLocus()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("姿态角"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionEulerAngle()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("Error"), this);
  connect(newAct, SIGNAL(triggered()), this, SLOT(onActionErrorFigure()));
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(220, 250, 255);");
  pToolBar->insertSeparator(newAct);
}

/*******************************************************
 * @brief 菜单响应函数，显示融合定位模块
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionLocation()
{
  this->processPreShow(m_pWdgLocation);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 菜单响应函数，显示融合数据信息方差
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionVariance()
{
  this->processPreShow(m_pWdgVariance);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 菜单响应函数，显示算法比较画面
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionAlgorithm()
{
  this->processPreShow(m_pWdgAlgorithm);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 菜单响应函数，显示2D/3D位置轨迹
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionLocus()
{
  this->processPreShow(m_pWdgLocus);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 菜单响应函数，显示姿态角画面
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionEulerAngle()
{
  this->processPreShow(m_pWdgEulerAngle);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 菜单响应函数，显示误差图
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionErrorFigure()
{
  this->processPreShow(m_pWdgErrorFigure);
  m_pWdgCurrent->show();
}

/*******************************************************
 * @brief 创建Perception模块对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createPerceptionToolBar()
{
  QToolBar *pToolBar = this->addToolBar(PERCEPTION_TOOL_BAR);

  QAction *newAct = new QAction(tr("2D"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPerception2D);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(250, 220, 255);");

  newAct = new QAction(tr("3D"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPerception3D);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(250, 220, 255);");
  pToolBar->insertSeparator(newAct);

  newAct = new QAction(tr("REPLAY"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPerceptionReplay);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(250, 220, 255);");
  pToolBar->insertSeparator(newAct);
}

/*******************************************************
 * @brief 以2D的方式实时显示当前数据、状态
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionPerception2D()
{
  this->processPreShow(m_pWdgPerception);
  m_pWdgPerception->showPerception(QPerceptionWidget::Perception2D);
}

/*******************************************************
 * @brief 以3D的方式实时显示当前数据、状态
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionPerception3D()
{
  this->processPreShow(m_pWdgPerception);
  m_pWdgPerception->showPerception(QPerceptionWidget::Perception3D);
}

/*******************************************************
 * @brief 回放perception数据
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionPerceptionReplay()
{
  this->processPreShow(m_pWdgPerception);
  m_pWdgPerception->showType(QPerceptionWidget::Replay);
}

/*******************************************************
 * @brief 创建Planning模块对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createPlanningToolBar()
{
  QToolBar *pToolBar = this->addToolBar(PLANNING_TOOL_BAR);

  QAction *newAct = new QAction(tr("LIVE\nDISPLAY"), this);
  connect(newAct, &QAction::triggered, this, &QDebugToolMainWnd::onActionPlanningLiveDisplay);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(255, 250, 220);");

  newAct = new QAction(tr("REPLAY"), this);
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
  this->processPreShow(m_pWdgPlanning);
  m_pWdgPlanning->showType(QPlanningWidget::LiveDisplay, "");
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
    fs::path fsPath = fs::current_path();
    fsPath /= "PlanningData";
    strOpenPath = QString::fromStdString(fsPath.string());
  }
  this->processPreShow(m_pWdgPlanning);
  QString strPath = QFileDialog::getExistingDirectory(this, tr("Json Directory"),
                                    strOpenPath,
                                    QFileDialog::ShowDirsOnly
                                    | QFileDialog::DontResolveSymlinks);
  if (!strPath.isEmpty()) {
    int index = strPath.lastIndexOf('/');
    strOpenPath = strPath.mid(0, index);
    m_pWdgPlanning->showType(QPlanningWidget::Replay, strPath);
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
}

/*******************************************************
 * @brief 放大显示当前调试画面，如果当前画面无放大功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewZoomIn()
{
  if (m_pWdgPlanning->isVisible()) {
    m_pWdgPlanning->setViewResolution(1);
  }
  if (m_pWdgPerception->isVisible()) {
    m_pWdgPerception->setViewResolution(1);
  }
}

/*******************************************************
 * @brief 缩小显示当前调试画面，如果当前画面无缩小功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewZoomOut()
{
  if (m_pWdgPlanning->isVisible()) {
    m_pWdgPlanning->setViewResolution(-1);
  }
  if (m_pWdgPerception->isVisible()) {
    m_pWdgPerception->setViewResolution(-1);
  }
}

/*******************************************************
 * @brief 复位当前显示画面到原始显示大小，如果当前画面无放大、缩小功能，忽略此操作
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionViewReset()
{
  if (m_pWdgPlanning->isVisible()) {
    m_pWdgPlanning->setViewResolution(0);
  }
  if (m_pWdgPerception->isVisible()) {
    m_pWdgPerception->setViewResolution(0);
  }
}

/*******************************************************
 * @brief 创建Setting对应的ToolBar
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::createSettingToolBar()
{
  QToolBar *pToolBar = this->addToolBar(SETTINT_TOOL_BAR);

  m_bFlagSaveJson = false;
  m_pActionSaveJson = new QAction(tr("SAVE\nJSON"), this);
  connect(m_pActionSaveJson, &QAction::triggered,
          this, &QDebugToolMainWnd::onActionSettingSaveJson);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionSaveJson);
  QWidget *pWdgAction = pToolBar->widgetForAction(m_pActionSaveJson);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - 4, TOOL_BAR_ACTION_SIZE - 4);
  pWdgAction->setStyleSheet("background-color: rgb(200, 255, 200);");
}

/*******************************************************
 * @brief 设置是否保存Json到文件
 * @param

 * @return
********************************************************/
void QDebugToolMainWnd::onActionSettingSaveJson()
{
  m_bFlagSaveJson = !m_bFlagSaveJson;
  if (m_bFlagSaveJson) {
    m_pActionSaveJson->setText(tr("STOP\nSAVE"));
  }
  else {
    m_pActionSaveJson->setText(tr("SAVE\nJSON"));
  }
  m_pActionMenuSaveJson->setChecked(m_bFlagSaveJson);
  //ReadDataManager::instance()->setSaveJson(m_bFlagSaveJson);
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
