#ifndef QSTUDIOWINDOW_H
#define QSTUDIOWINDOW_H

#include <QMainWindow>
#include <boost/filesystem.hpp>

class QCentralWidget;

class QStudioWindow : public QMainWindow
{
  Q_OBJECT
public:
  explicit QStudioWindow(QWidget *parent = nullptr);

protected:
  void createMenu();
  void createToolBar();

signals:

protected slots:
  void onActionFileOpenDir();
  void onActionFileSaveData();
  void onActionLiving();

private:
  QCentralWidget *m_pWdgCentral;

  QAction *n_pActionSaveData;
  QAction *n_pActionLiving;

  boost::filesystem::path m_fsPath;
};

#endif // QSTUDIOWINDOW_H
