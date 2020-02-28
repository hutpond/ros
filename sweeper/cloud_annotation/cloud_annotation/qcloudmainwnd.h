#ifndef QCLOUDMAINWND_H
#define QCLOUDMAINWND_H

#include <QMainWindow>

class QPointsShowWidget;
class QCloudPoints;

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

private:
  QPointsShowWidget *m_pWdgPointsShow;
  QCloudPoints *m_pObjPointsData;
};
#endif // QCLOUDMAINWND_H
