#include "qstudiowindow.h"

#include <QMenuBar>
#include <QFileDialog>
#include <QDockWidget>
#include <QTextBrowser>
#include "qcentralwidget.h"

static const char *WND_TITLE = "Decision Studio 1.2";

QStudioWindow::QStudioWindow(QWidget *parent) : QMainWindow(parent)
{
  m_fsPath = getenv("HOME");
  m_fsPath /= "DecisionData";

  this->createMenu();
  this->createToolBar();

  m_pWdgCentral = new QCentralWidget(this);
  m_pWdgCentral->createSavePath(m_fsPath);
  m_pWdgCentral->setFunciton(std::bind(&QStudioWindow::setData, this, std::placeholders::_1));
  this->setCentralWidget(m_pWdgCentral);

  connect(m_pWdgCentral, SIGNAL(replayFileName(const QString &)),
          this, SLOT(onSetWindowTitle(const QString &)));
  this->onSetWindowTitle();

  QDockWidget *pDockWdg = new QDockWidget("", this);
  pDockWdg->setFeatures(QDockWidget::AllDockWidgetFeatures);
  QWidget *pDockTitle = new QWidget;
  pDockTitle->setStyleSheet("background-color: rgb(114, 159, 207);");
  pDockWdg->setTitleBarWidget(pDockTitle);
  this->addDockWidget(Qt::BottomDockWidgetArea, pDockWdg);

  QTabWidget *pTabWidget = new QTabWidget(this);
  pDockWdg->setWidget(pTabWidget);
  m_pTextBrowserDebug = new QTextBrowser(this);
  pTabWidget->addTab(m_pTextBrowserDebug, QStringLiteral("Debug"));

  this->statusBar();
}

void QStudioWindow::createMenu()
{
  QMenu *menu = menuBar()->addMenu(QStringLiteral("&File"));
  menu->addAction(QStringLiteral("Open Dir"), this, SLOT(onActionFileOpenDir()));
  n_pActionLiving = menu->addAction(
        QStringLiteral("Living"), this, SLOT(onActionLiving())
        );
  n_pActionLiving->setCheckable(true);
  n_pActionLiving->setChecked(true);
  n_pActionSaveData = menu->addAction(
        QStringLiteral("Save Data"), this, SLOT(onActionFileSaveData())
        );
  n_pActionSaveData->setCheckable(true);
  n_pActionSaveData->setChecked(false);

  menu = menuBar()->addMenu(QStringLiteral("&Edit"));
  menu = menuBar()->addMenu(QStringLiteral("&View"));
  menu = menuBar()->addMenu(QStringLiteral("&Help"));
}

void QStudioWindow::createToolBar()
{

}

void QStudioWindow::onActionFileOpenDir()
{
  QString strPath = QFileDialog::getExistingDirectory(this, tr("Json Directory"),
                                    QString::fromStdString(m_fsPath.string()),
                                    QFileDialog::ShowDirsOnly
                                    | QFileDialog::DontResolveSymlinks);
  if (!strPath.isEmpty()) {
    m_pWdgCentral->openReplayDir(strPath);
  }
}

void QStudioWindow::onActionFileSaveData()
{
  bool checked = n_pActionSaveData->isChecked();
  m_pWdgCentral->setSaveDataFlag(checked);
}

void QStudioWindow::onActionLiving()
{
  bool checked = n_pActionLiving->isChecked();
  m_pWdgCentral->setLivingFlag(checked);

  this->onSetWindowTitle();
}

void QStudioWindow::onSetWindowTitle(const QString &text)
{
  QString title = WND_TITLE;
  if (n_pActionLiving->isChecked()) {
    title += " - Liveing";
  }
  else {
    title += " - Replay";
  }
  if (!text.isEmpty()) {
    title += " - ";
    int index = text.lastIndexOf('/');
    title += text.mid(index + 1);
  }
  this->setWindowTitle(title);
}

void QStudioWindow::setData(const decision_studio::ads_DecisionData4Debug &data)
{
  m_pTextBrowserDebug->setPlainText(QString::fromStdString(data.debug_info));
}
