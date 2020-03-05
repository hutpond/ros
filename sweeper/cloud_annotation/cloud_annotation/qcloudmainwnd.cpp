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

QCloudMainWnd::QCloudMainWnd(QWidget *parent)
  : QMainWindow(parent)
{
  m_pObjPointsData = new QCloudPoints(this);
  m_pWdgPointsShow = new QPointsShowWidget(*m_pObjPointsData, this);

  this->setCentralWidget(m_pWdgPointsShow);

  this->createMenuBar();
  this->createToolBar();
  this->createDockWidget();
  this->setStatusBar(new QStatusBar);

  connect(m_pWdgPointsShow, &QPointsShowWidget::message, this, &QCloudMainWnd::onPlotMessage);
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
  QAction *actionOpen = new QAction(tr("&Open"), this);
  actionOpen->setStatusTip(tr("Open a cloud points file"));
  connect(actionOpen, &QAction::triggered, this, &QCloudMainWnd::open);
  menuFile->addAction(actionOpen);

  QAction *actionExit = new QAction(tr("&Exit"), this);
  actionExit->setStatusTip(tr("Exit the program"));
  connect(actionExit, &QAction::triggered, this, &QCloudMainWnd::close);
  menuFile->addAction(actionExit);

  // menu edit
  QAction *actionReset = new QAction(tr("&Reset"), this);
  actionOpen->setStatusTip(tr("Reset the state of Cloud Points"));
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
  QActionGroup *group= new QActionGroup(this);
  action = leftToolBar->addAction(QStringLiteral("路沿"), m_pWdgPointsShow, &QPointsShowWidget::onRoadSideClck);
  action->setToolTip(QStringLiteral("选择路边沿点"));
  action->setCheckable(true);
  action->setChecked(false);
  group->addAction(action);

  action = leftToolBar->addAction(QStringLiteral(""), m_pWdgPointsShow, &QPointsShowWidget::onCrossWalkClck);
  action->setToolTip(QStringLiteral("选择人行道位置"));
  action->setCheckable(true);
  action->setChecked(false);
  group->addAction(action);

  this->addToolBar(Qt::LeftToolBarArea, leftToolBar);
}

void QCloudMainWnd::createDockWidget()
{
  this->addDockWidget(Qt::LeftDockWidgetArea, new QDockWidget(QStringLiteral("HdMap")));

  // bottom
  m_pTextBrowser = new QTextBrowser;
  QDockWidget *dockWidget = new QDockWidget(QStringLiteral("Console"));
  dockWidget->setWidget(m_pTextBrowser);
  this->addDockWidget(Qt::BottomDockWidgetArea, dockWidget);
}

void QCloudMainWnd::open()
{
  QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Cloud Points File"), getenv("HOME"), tr("Cloud Points Files (*.ply)"));

  m_pObjPointsData->openFile(fileName);
  m_pWdgPointsShow->update();
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
