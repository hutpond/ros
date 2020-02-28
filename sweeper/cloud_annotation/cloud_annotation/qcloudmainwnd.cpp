#include "qcloudmainwnd.h"

#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>

#include "qpointsshowwidget.h"
#include "qcloudpoints.h"

QCloudMainWnd::QCloudMainWnd(QWidget *parent)
  : QMainWindow(parent)
{
  this->createMenuBar();
  this->createToolBar();
  this->createDockWidget();

  m_pObjPointsData = new QCloudPoints(this);

  m_pWdgPointsShow = new QPointsShowWidget(*m_pObjPointsData, this);
  this->setCentralWidget(m_pWdgPointsShow);
}

QCloudMainWnd::~QCloudMainWnd()
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
  this->addToolBar(Qt::LeftToolBarArea, new QToolBar);
}

void QCloudMainWnd::createDockWidget()
{
  this->addDockWidget(Qt::LeftDockWidgetArea, new QDockWidget(QStringLiteral("HdMap")));
  this->addDockWidget(Qt::BottomDockWidgetArea, new QDockWidget(QStringLiteral("Console")));
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
  m_pWdgPointsShow->reset();
}
