#ifndef QCLOUDMAINWND_H
#define QCLOUDMAINWND_H

#include <QMainWindow>
#include "GlobalDefine.h"

class QPointsShowWidget;
class QHdMapWidget;
class QTextBrowser;

struct ProjectInfo
{
  int version_;
  QString path_name_;
  QString project_name_;
  QString point_cloud_file_;
  QString reference_file_;
  Point point_cloud_origin_;
};

class QCloudMainWnd : public QMainWindow
{
  Q_OBJECT

public:
  QCloudMainWnd(QWidget *parent = nullptr);
  ~QCloudMainWnd();

protected:
  virtual void resizeEvent(QResizeEvent *) final;

  void createMenuBar();
  void createToolBar();
  void createDockWidget();

  QString projectPath();
  QString projectSubName();
  void saveProjectInfo();
  bool parseProjectInfo();

  void setWndTitle();
  void loadFiles();

protected slots:
  // &file
  void newProject();
  void openProject();
  void saveProject();
  void closeProject();
  // &edit
  void reset();

  void onPlotMessage(const QString &);

private:
  QPointsShowWidget *m_pWdgPointsShow;
  QHdMapWidget *m_pWdgHdMap;
  QTextBrowser *m_pTextBrowser;

  ProjectInfo m_projectInfo;
};
#endif // QCLOUDMAINWND_H
