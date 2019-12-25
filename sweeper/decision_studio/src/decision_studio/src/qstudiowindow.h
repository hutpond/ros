#ifndef QSTUDIOWINDOW_H
#define QSTUDIOWINDOW_H

#include <QMainWindow>
#include <boost/filesystem.hpp>
#include "decision_studio/ads_DecisionData4Debug.h"

class QCentralWidget;
class QTextBrowser;

class QStudioWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit QStudioWindow(QWidget *parent = nullptr);

protected:
  void createMenu();
  void createToolBar();

  void setData(const decision_studio::ads_DecisionData4Debug &);

signals:

protected slots:
  void onActionFileOpenDir();
  void onActionFileSaveData();
  void onActionLiving();

  void onSetWindowTitle(const QString & = "");

private:
  QCentralWidget *m_pWdgCentral;
  QTextBrowser *m_pTextBrowserDebug;

  QAction *n_pActionSaveData;
  QAction *n_pActionLiving;

  boost::filesystem::path m_fsPath;
};

#endif // QSTUDIOWINDOW_H
