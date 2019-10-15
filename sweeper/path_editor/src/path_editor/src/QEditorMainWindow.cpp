/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QEditorMainWindow.cpp
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 主界面
********************************************************/
#include <functional>
#include <QAction>
#include <QToolBar>
#include <QDebug>
#include <QSettings>
#include <QDockWidget>
#include <QListWidget>
#include <QTextEdit>
#include <QStatusBar>
#include <QTextBrowser>
#include <QDir>
#include <QFileDialog>
#include "QEditorMainWindow.h"
#include "QReadDataRosObject.h"
#include "QProjectObject.h"
#include "QProjectManagerWidget.h"
#include "QDrawPathWidget.h"
#include "QPanelWidget.h"

static constexpr int TOOL_BAR_ACTION_SIZE = 50;
static constexpr int TOOL_BAR_ACTION_SPACE = 8;
static const QString WND_TITLE = "Path Editor";

QEditorMainWindow::QEditorMainWindow(QWidget *parent)
  : QMainWindow(parent)
  , m_pWdgDrawPath(nullptr)
  , m_pDockWdgProjectManager(nullptr)
  , m_pWdgPanel(nullptr)
  , m_pTextBrowserOutput(nullptr)
{
  this->createToolBar();
  QStatusBar *pBar = new QStatusBar;
  this->setStatusBar(pBar);
  pBar->showMessage("ready");

  m_pObjProject = new QProjectObject(this);

  m_pObjReadDataRos = new QReadDataRosObject(this);
  connect(m_pObjReadDataRos, SIGNAL(imuData(const path_editor::ads_ins_data::ConstPtr &)),
          m_pObjProject, SLOT(onSetImuData(const path_editor::ads_ins_data::ConstPtr &)));

  m_pObjReadDataRos->startSubscribe();
}

QEditorMainWindow::~QEditorMainWindow()
{
  m_pObjReadDataRos->stopSubscribe();
}

/*******************************************************
 * @brief 创建Tool Bar
 * @param

 * @return
********************************************************/
void QEditorMainWindow::createToolBar()
{
  QToolBar *pToolBar = this->addToolBar(QStringLiteral("File"));

  QAction *newAct = new QAction(QIcon(":/image/new.svg"), "", this);
  newAct->setIconText(QStringLiteral("new"));
  connect(newAct, &QAction::triggered, this, &QEditorMainWindow::onActionNew);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(newAct);
  QWidget *pWdgAction = pToolBar->widgetForAction(newAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);

  QAction *openAct = new QAction(QIcon(":/image/open.svg"), "", this);
  openAct->setIconText(QStringLiteral("open"));
  connect(openAct, &QAction::triggered, this, &QEditorMainWindow::onActionOpen);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(openAct);
  pWdgAction = pToolBar->widgetForAction(openAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);

  m_pActionSave = new QAction(QIcon(":/image/save.svg"), "", this);
  m_pActionSave->setIconText(QStringLiteral("save"));
  m_pActionSave->setEnabled(false);
  connect(m_pActionSave, &QAction::triggered, this, &QEditorMainWindow::onActionSave);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionSave);
  pWdgAction = pToolBar->widgetForAction(m_pActionSave);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);

  m_pActionBuild = new QAction(QIcon(":/image/build.svg"), "", this);
  m_pActionBuild->setIconText(QStringLiteral("build"));
  m_pActionBuild->setEnabled(false);
  connect(m_pActionBuild, &QAction::triggered, this, &QEditorMainWindow::onActionBuild);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionBuild);
  pWdgAction = pToolBar->widgetForAction(m_pActionBuild);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);

  m_pActionClose = new QAction(QIcon(":/image/close.svg"), "", this);
  m_pActionClose->setIconText(QStringLiteral("close"));
  m_pActionClose->setEnabled(false);
  connect(m_pActionClose, &QAction::triggered, this, &QEditorMainWindow::onActionClose);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(m_pActionClose);
  pWdgAction = pToolBar->widgetForAction(m_pActionClose);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);

  QAction *settingAct = new QAction(QIcon(":/image/setting.svg"), "", this);
  settingAct->setIconText(QStringLiteral("setting"));
  connect(settingAct, &QAction::triggered, this, &QEditorMainWindow::onActionSetting);
  pToolBar->setMinimumHeight(TOOL_BAR_ACTION_SIZE);
  pToolBar->addAction(settingAct);
  pWdgAction = pToolBar->widgetForAction(settingAct);
  pWdgAction->setMinimumSize(TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE,
                             TOOL_BAR_ACTION_SIZE - TOOL_BAR_ACTION_SPACE);
}

/**
 * @brief toolbar new 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::createWidget()
{
  if (m_pWdgDrawPath == nullptr) {
    m_pWdgDrawPath = new QDrawPathWidget(*m_pObjProject);
    this->setCentralWidget(m_pWdgDrawPath);
  }
  this->centralWidget()->show();

  if (m_pWdgPanel == nullptr) {
    m_pDockWdgPanel = new QDockWidget(tr("Panel"), this);
    m_pWdgPanel = new QPanelWidget(m_pDockWdgPanel);
    m_pDockWdgPanel->setWidget(m_pWdgPanel);
    this->addDockWidget(Qt::RightDockWidgetArea, m_pDockWdgPanel);

    connect(m_pWdgPanel, &QPanelWidget::operate, m_pWdgDrawPath, &QDrawPathWidget::onOperate);
  }
  m_pDockWdgPanel->show();

  if (m_pDockWdgProjectManager == nullptr) {
    m_pDockWdgProjectManager = new QDockWidget(tr("map"), this);
    m_pWdgProjectManager = new QProjectManagerWidget(m_pObjProject, m_pDockWdgProjectManager);
    m_pDockWdgProjectManager->setWidget(m_pWdgProjectManager);
    this->addDockWidget(Qt::LeftDockWidgetArea, m_pDockWdgProjectManager);

    connect(m_pWdgProjectManager, &QProjectManagerWidget::addBoundary,
            m_pWdgDrawPath, &QDrawPathWidget::onAddBoundary);
    connect(m_pWdgProjectManager, &QProjectManagerWidget::operateSignal,
            m_pWdgDrawPath, &QDrawPathWidget::onOperateSignal);
  }
  m_pDockWdgProjectManager->show();

  if (m_pTextBrowserOutput == nullptr) {
    m_pTextBrowserOutput = new QTextBrowser;
    m_pDockWdgOutput = new QDockWidget(tr("Output"), this);
    m_pDockWdgOutput->setFeatures(QDockWidget::AllDockWidgetFeatures);
    m_pDockWdgOutput->setWidget(m_pTextBrowserOutput);
    this->addDockWidget(Qt::BottomDockWidgetArea, m_pDockWdgOutput);
  }
  m_pDockWdgOutput->show();
}

/**
 * @brief toolbar new 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionNew()
{
  QCreateProjectDialog dlg(this);
  QRect rect = this->rect();
  QPoint pt = rect.center();
  rect.setSize(QSize(rect.width() * 0.5, rect.height() * 0.3));
  rect.moveCenter(pt);
  dlg.setGeometry(rect);
  dlg.setWindowTitle(QStringLiteral("Create Project"));

  if (dlg.exec() == QDialog::Accepted) {
    m_pActionSave->setEnabled(true);
    m_pActionBuild->setEnabled(true);
    m_pActionClose->setEnabled(true);

    QString path, name;
    dlg.getProjectName(path, name);
    m_pObjProject->createProject(path, name);

    this->createWidget();
    QString title = WND_TITLE + " - " + name;
    this->setWindowTitle(title);
    m_pWdgProjectManager->doUpdate();
  }
}

/**
 * @brief toolbar open 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionOpen()
{
  QString project = QFileDialog::getExistingDirectory(
        this, tr("Open Directory"),
        QDir::currentPath(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks
        );
  if (project.isEmpty()) {
    return;
  }

  this->createWidget();
  m_pObjProject->openProject(project);
  m_pWdgProjectManager->doUpdate();
  m_pWdgDrawPath->onOperate(QPanelWidget::TypeZoomReset);
  m_pActionSave->setEnabled(true);
  m_pActionBuild->setEnabled(true);
  m_pActionClose->setEnabled(true);

  int index = project.lastIndexOf('/');
  QString title = project.mid(index + 1);
  title = WND_TITLE + " - " + title;
  this->setWindowTitle(title);
}

/**
 * @brief toolbar save 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionSave()
{
  m_pObjProject->saveProject();
}

/**
 * @brief toolbar build 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionBuild()
{
  m_pObjProject->buildProject();
}

/**
 * @brief toolbar close 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionClose()
{
  m_pObjProject->closeProject();
  this->centralWidget()->close();
  m_pDockWdgProjectManager->close();
  m_pDockWdgPanel->close();
  m_pDockWdgOutput->close();

  m_pActionSave->setEnabled(false);
  m_pActionBuild->setEnabled(false);
  m_pActionClose->setEnabled(false);

  this->setWindowTitle(WND_TITLE);
}

/**
 * @brief toolbar setting 响应槽函数
 * @param
 *
 * @return
 */
void QEditorMainWindow::onActionSetting()
{
}

