#include "qcloudmainwnd.h"

#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QTextBrowser>
#include <QStatusBar>

#include "qpointsshowwidget.h"
#include "qcloudpoints.h"
#include "QHdMapWidget.h"
#include "QProjectDialog.h"

QCloudMainWnd::QCloudMainWnd(QWidget *parent)
  : QMainWindow(parent)
{
  m_pWdgPointsShow = new QPointsShowWidget(this);

  this->setCentralWidget(m_pWdgPointsShow);

  this->createMenuBar();
  this->createToolBar();
  this->createDockWidget();
  this->setStatusBar(new QStatusBar);

  connect(m_pWdgPointsShow, &QPointsShowWidget::message,
          this, &QCloudMainWnd::onPlotMessage);
  connect(m_pWdgPointsShow, SIGNAL(clickedPoint(const Point &)),
          m_pWdgHdMap, SLOT(onAddPoint(const Point &)));
}

QCloudMainWnd::~QCloudMainWnd()
{
}

void QCloudMainWnd::resizeEvent(QResizeEvent *)
{
}

void QCloudMainWnd::createMenuBar()
{
  QMenuBar *menuBar = this->menuBar();

  // menu
  QMenu *menuFile = menuBar->addMenu(tr("&File"));
  QMenu *menuEdit = menuBar->addMenu(tr("&Edit"));
  menuBar->addMenu(tr("&Help"));

  // menu file
  QAction *actionNewProject = new QAction(tr("&New Project"), this);
  actionNewProject->setStatusTip(tr("Create new project"));
  connect(actionNewProject, &QAction::triggered, this, &QCloudMainWnd::newProject);
  menuFile->addAction(actionNewProject);

  QAction *actionOpenProject = new QAction(tr("&Open Project"), this);
  actionOpenProject->setStatusTip(tr("Open a exist project"));
  connect(actionOpenProject, &QAction::triggered, this, &QCloudMainWnd::openProject);
  menuFile->addAction(actionOpenProject);

  QAction *actionLoadFile = new QAction(tr("&Load Cloud File"), this);
  actionLoadFile->setStatusTip(tr("Load cloud file of ply"));
  connect(actionLoadFile, &QAction::triggered, this, &QCloudMainWnd::loadPlyFile);
  menuFile->addAction(actionLoadFile);

  QAction *actionCloseProject = new QAction(tr("&Close Project"), this);
  actionCloseProject->setStatusTip(tr("Close the opening project"));
  connect(actionCloseProject, &QAction::triggered, this, &QCloudMainWnd::closeProject);
  menuFile->addAction(actionCloseProject);

  QAction *actionExit = new QAction(tr("&Exit"), this);
  actionExit->setStatusTip(tr("Exit the program"));
  connect(actionExit, &QAction::triggered, this, &QCloudMainWnd::close);
  menuFile->addAction(actionExit);

  // menu edit
  QAction *actionReset = new QAction(tr("&Reset"), this);
  actionReset->setStatusTip(tr("Reset the state of Cloud Points"));
  connect(actionReset, &QAction::triggered, this, &QCloudMainWnd::reset);
  menuEdit->addAction(actionReset);
}

void QCloudMainWnd::createToolBar()
{
  this->addToolBar(Qt::TopToolBarArea, new QToolBar);

  // left show
  QToolBar *leftToolBar = new QToolBar;
  QAction *action = leftToolBar->addAction(
        QIcon(":/image/ccViewIso1.png"), "", m_pWdgPointsShow, &QPointsShowWidget::onReset);
  action->setToolTip(QStringLiteral("重置点云显示"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewYpos"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowFront);
  action->setToolTip(QStringLiteral("显示前端面"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewYneg"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowBack);
  action->setToolTip(QStringLiteral("显示后端面"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewZpos"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowTop);
  action->setToolTip(QStringLiteral("显示上端面"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewZneg"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowBottom);
  action->setToolTip(QStringLiteral("显示下端面"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewXneg"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowRight);
  action->setToolTip(QStringLiteral("显示右侧面"));

  action = leftToolBar->addAction(
        QIcon(":/image/ccViewXpos"), "", m_pWdgPointsShow, &QPointsShowWidget::onShowLeft);
  action->setToolTip(QStringLiteral("显示左端面"));

  leftToolBar->addSeparator();

  // left clicked
  action = leftToolBar->addAction(QIcon(":/image/clicked_select.svg"), "",
                                  m_pWdgPointsShow, &QPointsShowWidget::onClickedSelect);
  action->setToolTip(QStringLiteral("取消选点操作"));
  action->setCheckable(true);
  action->setChecked(false);

  this->addToolBar(Qt::LeftToolBarArea, leftToolBar);
}

void QCloudMainWnd::createDockWidget()
{
  QDockWidget *dockWidget = new QDockWidget(QStringLiteral("HdMap"));
  m_pWdgHdMap = new QHdMapWidget;
  m_pWdgHdMap->setEnabled(false);
  dockWidget->setWidget(m_pWdgHdMap);
  this->addDockWidget(Qt::LeftDockWidgetArea, dockWidget);

  // bottom
  m_pTextBrowser = new QTextBrowser;
  dockWidget = new QDockWidget(QStringLiteral("Console"));
  dockWidget->setWidget(m_pTextBrowser);
  this->addDockWidget(Qt::BottomDockWidgetArea, dockWidget);
}

void QCloudMainWnd::newProject()
{
  QProjectDialog dlg;
  if (dlg.exec() == QDialog::Accepted) {
    this->closeProject();

    m_strProjectPath = dlg.projectPath();
    m_strProjectName = dlg.projectName();

    QDir dir(m_strProjectPath);
    dir.mkpath(m_strProjectName);

    m_pWdgHdMap->setEnabled(true);
  }
}

void QCloudMainWnd::openProject()
{
  QString pathName = QFileDialog::getExistingDirectory(
        this, tr("Open Project"),
        getenv("HOME"),
        QFileDialog::ShowDirsOnly
        | QFileDialog::DontResolveSymlinks);
  if (pathName.isEmpty()) {
    return;
  }

  this->closeProject();
  int index = pathName.lastIndexOf('/');
  m_strProjectPath = pathName.mid(0, index + 1);
  m_strProjectName = pathName.mid(index + 1);
  m_pWdgHdMap->setEnabled(true);
}

void QCloudMainWnd::loadPlyFile()
{
  if (m_strProjectName.isEmpty() || m_strProjectPath.isEmpty()) {
    return;
  }
  QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Cloud Points File"), getenv("HOME"), tr("Cloud Points Files (*.ply)"));

  QCloudPoints::instance().openFile(fileName);
  m_pWdgPointsShow->update();
}

void QCloudMainWnd::closeProject()
{
  m_pWdgHdMap->setEnabled(false);
  m_pTextBrowser->clear();

  m_strProjectPath.clear();
  m_strProjectName.clear();
  m_pWdgHdMap->clear();
  QCloudPoints::instance().clear();
}

void QCloudMainWnd::reset()
{
  m_pWdgPointsShow->onReset();
}

void QCloudMainWnd::onPlotMessage(const QString &msg)
{
  QString text = m_pTextBrowser->toPlainText();
  if (!text.isEmpty()) {
    text.push_back('\n');
  }
  text += msg;
  m_pTextBrowser->setPlainText(text);
  m_pTextBrowser->moveCursor(QTextCursor::End);
}
