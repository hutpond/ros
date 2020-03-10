#ifndef QCLOUDMAINWND_H
#define QCLOUDMAINWND_H

#include <QMainWindow>

class QPointsShowWidget;
class QHdMapWidget;
class QTextBrowser;

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

protected slots:
  // &file
  void newProject();
  void openProject();
  void loadPlyFile();
  void closeProject();
  // &edit
  void reset();

  void onPlotMessage(const QString &);

private:
  QPointsShowWidget *m_pWdgPointsShow;
  QHdMapWidget *m_pWdgHdMap;
  QTextBrowser *m_pTextBrowser;

  QString m_strProjectPath;
  QString m_strProjectName;
};
#endif // QCLOUDMAINWND_H
