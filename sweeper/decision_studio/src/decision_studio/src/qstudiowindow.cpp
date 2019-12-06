#include <QMenuBar>
#include "qstudiowindow.h"
#include "qcentralwidget.h"

QStudioWindow::QStudioWindow(QWidget *parent) : QMainWindow(parent)
{
  this->createMenu();
  this->createToolBar();

  m_pWdgCentral = new QCentralWidget(this);
  this->setCentralWidget(m_pWdgCentral);
}

void QStudioWindow::createMenu()
{
  QMenu *menu = menuBar()->addMenu(QStringLiteral("&File"));
  menu = menuBar()->addMenu(QStringLiteral("&Edit"));
  menu = menuBar()->addMenu(QStringLiteral("&View"));
  menu = menuBar()->addMenu(QStringLiteral("&Help"));
}

void QStudioWindow::createToolBar()
{

}
