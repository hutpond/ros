#ifndef QCLOUDMAINWND_H
#define QCLOUDMAINWND_H

#include <QMainWindow>

class QPointsShowWidget;
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
  void open();
  // &edit
  void reset();

  void onPlotMessage(const QString &);

private:
  QPointsShowWidget *m_pWdgPointsShow;

  QTextBrowser *m_pTextBrowser;
};
#endif // QCLOUDMAINWND_H
