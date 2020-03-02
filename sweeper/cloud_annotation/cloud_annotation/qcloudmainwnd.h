#ifndef QCLOUDMAINWND_H
#define QCLOUDMAINWND_H

#include <QMainWindow>

class QPointsShowWidget;
class QCloudPoints;
class QTextBrowser;

class QCloudMainWnd : public QMainWindow
{
  Q_OBJECT

public:
  QCloudMainWnd(QWidget *parent = nullptr);
  ~QCloudMainWnd();

protected:
  void createMenuBar();
  void createToolBar();
  void createDockWidget();

protected slots:
  // &file
  void open();
  // &edit
  void reset();

  void onPlotMessage(const QString &);

private:
  QPointsShowWidget *m_pWdgPointsShow;
  QCloudPoints *m_pObjPointsData;

  QTextBrowser *m_pTextBrowser;
};
#endif // QCLOUDMAINWND_H
