#include "qcloudmainwnd.h"

#include <fstream>
#include <thread>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QToolBar>
#include <QDockWidget>
#include <QFileDialog>
#include <QTextBrowser>
#include <QStatusBar>
#include <jsoncpp/json/json.h>

#include "qpointsshowwidget.h"
#include "qcloudpoints.h"
#include "QHdMapWidget.h"
#include "QProjectDialog.h"
#include "QWaitingDialog.h"
#include "QDisplayAreaDialog.h"
#include "QLanelet2Data.h"

QCloudMainWnd::QCloudMainWnd(QWidget *parent)
  : QMainWindow(parent)
{
  QTabWidget *tab_widget = new QTabWidget(this);
  m_pWdgPointsShow = new QPointsShowWidget(this);
  tab_widget->addTab(m_pWdgPointsShow, QStringLiteral("Point Cloud"));

  this->setCentralWidget(tab_widget);

  this->createMenuBar();
  this->createToolBar();
  this->createDockWidget();
  this->setStatusBar(new QStatusBar);

  connect(m_pWdgPointsShow, &QPointsShowWidget::message,
          this, &QCloudMainWnd::onPlotMessage);
  connect(m_pWdgPointsShow, SIGNAL(clickedPoint(const Point &)),
          m_pWdgHdMap, SLOT(onAddPoint(const Point &)));

  connect(&QCloudPoints::instance(), SIGNAL(updateData()),
          m_pWdgPointsShow, SLOT(update()));

  m_projectInfo.version_ = 0;
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
  QMenu *menuDisplay = menuBar->addMenu(tr("&dispaly"));
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

  QAction *actionSaveProject = new QAction(tr("&Save Project"), this);
  actionSaveProject->setStatusTip(tr("Save project"));
  connect(actionSaveProject, &QAction::triggered, this, &QCloudMainWnd::saveProject);
  menuFile->addAction(actionSaveProject);

  QAction *actionCloseProject = new QAction(tr("&Close Project"), this);
  actionCloseProject->setStatusTip(tr("Close the opening project"));
  connect(actionCloseProject, &QAction::triggered, this, &QCloudMainWnd::closeProject);
  menuFile->addAction(actionCloseProject);

  QAction *actionExit = new QAction(tr("&Exit"), this);
  actionExit->setStatusTip(tr("Exit the program"));
  connect(actionExit, &QAction::triggered, this, &QCloudMainWnd::close);
  menuFile->addAction(actionExit);

  // menu view
  QAction *actionArea = new QAction(tr("&Area"), this);
  actionArea->setStatusTip(tr("Set display area"));
  connect(actionArea, &QAction::triggered, this, &QCloudMainWnd::setDisplayArea);
  menuDisplay->addAction(actionArea);
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

  action = leftToolBar->addAction(QIcon(":/image/point_cloud_show.svg"), "",
                                  m_pWdgPointsShow, &QPointsShowWidget::onClickedShowPointCloud);
  action->setToolTip(QStringLiteral("是否显示点云"));
  action->setCheckable(true);
  action->setChecked(true);

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

    m_projectInfo.path_name_ = dlg.projectPath();
    m_projectInfo.project_name_ = dlg.projectName();
    m_projectInfo.point_cloud_file_ = dlg.cloudPointName();
    m_projectInfo.reference_file_ = dlg.referenceName();
    m_projectInfo.point_cloud_origin_ = dlg.cloudPointOrigin();

    QDir dir(m_projectInfo.path_name_);
    dir.mkpath(m_projectInfo.project_name_);
    this->saveProjectInfo();

    QCloudPoints::instance().setOrigin(m_projectInfo.point_cloud_origin_);
    QCloudPoints::instance().openFile(m_projectInfo.point_cloud_file_);
    QCloudPoints::instance().openOpenDriverFile(m_projectInfo.reference_file_);
    m_pWdgHdMap->setEnabled(true);
    m_pWdgHdMap->updateHdMap();
    this->setWndTitle();
  }
}

void QCloudMainWnd::openProject()
{
  QString path = getenv("HOME");
  if (!path.endsWith('/')) {
    path.append('/');
  }
  path += "Documents/HdMap/";

  QString pathName = QFileDialog::getExistingDirectory(
        this, tr("Open Project"),
        path,
        QFileDialog::ShowDirsOnly
        | QFileDialog::DontResolveSymlinks);
  if (pathName.isEmpty()) {
    return;
  }

  this->closeProject();
  int index = pathName.lastIndexOf('/');
  m_projectInfo.path_name_ = pathName.mid(0, index + 1);
  m_projectInfo.project_name_ = pathName.mid(index + 1);

  if (this->parseProjectInfo()) {
    QCloudPoints::instance().setOrigin(m_projectInfo.point_cloud_origin_);
    this->loadFiles();
//    QCloudPoints::instance().openFile(m_projectInfo.point_cloud_file_);
//    QCloudPoints::instance().openOpenDriverFile(m_projectInfo.reference_file_);
    QString fileName = this->projectSubName()+ ".hdmap";
    m_pWdgHdMap->parseHdMapData(fileName);
    m_pWdgHdMap->setEnabled(true);
    this->setWndTitle();
  }
  else {
    this->closeProject();
  }
}

void QCloudMainWnd::saveProject()
{
  if (m_projectInfo.path_name_.isEmpty() || m_projectInfo.project_name_.isEmpty()) {
    return;
  }

  QString fileName = this->projectSubName()+ ".hdmap";
  m_pWdgHdMap->saveHdMapData(fileName);

  fileName = this->projectSubName()+ ".osm";
  HdMapRaw hdmap;
  QLanelet2Data::instance().setMapData(hdmap);
  QLanelet2Data::instance().saveOsmMapFile(fileName);
}

void QCloudMainWnd::closeProject()
{
  m_pWdgHdMap->setEnabled(false);
  m_pTextBrowser->clear();

  m_projectInfo.path_name_.clear();
  m_projectInfo.project_name_.clear();
  m_pWdgHdMap->clear();
  QCloudPoints::instance().clear();
  m_pWdgHdMap->updateHdMap();
  this->setWndTitle();
}

QString QCloudMainWnd::projectPath()
{
  QString pathName = m_projectInfo.path_name_;
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  pathName += m_projectInfo.project_name_;
  if (!pathName.endsWith('/')) {
    pathName.append('/');
  }
  return pathName;
}

QString QCloudMainWnd::projectSubName()
{
  QString name = this->projectPath();
  name += m_projectInfo.project_name_;
  return name;
}

void QCloudMainWnd::saveProjectInfo()
{
  Json::Value root;
  root["version"] = m_projectInfo.version_;
  root["point_cloud_file"] = m_projectInfo.point_cloud_file_.toStdString();
  root["reference_file"] = m_projectInfo.reference_file_.toStdString();
  root["longitude"] = m_projectInfo.point_cloud_origin_.x;
  root["latitude"] = m_projectInfo.point_cloud_origin_.y;
  root["height"] = m_projectInfo.point_cloud_origin_.z;

  QString fileName = this->projectSubName() + ".project";
  std::ofstream out(fileName.toLocal8Bit().data());
  Json::StreamWriterBuilder builder;
  std::unique_ptr<Json::StreamWriter> writer(builder.newStreamWriter());
  writer->write(root, &out);
  out.close();
}

bool QCloudMainWnd::parseProjectInfo()
{
  QString fileName = this->projectSubName()+ ".project";
  FILE *pf = fopen(fileName.toLocal8Bit().data(), "r");
  if (pf == NULL) {
    return false;
  }
  fseek(pf , 0 , SEEK_END);
  long size = ftell(pf);
  rewind(pf);
  char *buffer = (char*)malloc(size + 1);
  memset(buffer, 0, size + 1);
  if (buffer == NULL) {
    return false;
  }
  fread(buffer,1, size, pf);
  fclose(pf);

  Json::Value root;
  Json::Reader reader;
  if (!reader.parse(buffer, root)) {
    delete buffer;
    return false;
  }
  delete buffer;

  m_projectInfo.version_ = root["version"].asInt();
  m_projectInfo.point_cloud_file_ = QString::fromStdString(
        root["point_cloud_file"].asString());
  m_projectInfo.reference_file_ = QString::fromStdString(
        root["reference_file"].asString());
  m_projectInfo.point_cloud_origin_.x = root["longitude"].asDouble();
  m_projectInfo.point_cloud_origin_.y = root["latitude"].asDouble();
  m_projectInfo.point_cloud_origin_.z = root["height"].asDouble();

  return true;
}

void QCloudMainWnd::setDisplayArea()
{
  QDisplayAreaDialog dlg(this);
  if (QDialog::Accepted == dlg.exec()) {
    m_pWdgPointsShow->update();
  }
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


void QCloudMainWnd::setWndTitle()
{
  QString title("HdMap - 0.1");
  if (!m_projectInfo.project_name_.isEmpty()) {
    title += " - " + m_projectInfo.project_name_;
  }
  this->setWindowTitle(title);
}

void QCloudMainWnd::loadFiles()
{
  QWaitingDialog dlg(this);
  std::thread t([&]() {
    QCloudPoints::instance().openFile(m_projectInfo.point_cloud_file_);
    QCloudPoints::instance().openOpenDriverFile(m_projectInfo.reference_file_);
  });
  connect(&QCloudPoints::instance(), SIGNAL(loadFinished()),
          &dlg, SLOT(close()));
  t.detach();
  dlg.exec();
}
