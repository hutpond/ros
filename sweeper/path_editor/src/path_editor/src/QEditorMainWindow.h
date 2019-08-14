/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QEditorMainWindow.h
 * Author: liuzheng
 * Date: 2019/7/16
 * Description: 主界面
********************************************************/
#ifndef QEDITORMAINWINDOW_H
#define QEDITORMAINWINDOW_H

#include <QMainWindow>

class QReadDataObject;
class QReadDataRosObject;
class QProjectObject;
class QDrawPathWidget;
class QPanelWidget;
class QTextBrowser;

class QEditorMainWindow : public QMainWindow
{
  Q_OBJECT

public:
  QEditorMainWindow(QWidget *parent = 0);
  ~QEditorMainWindow();

protected:
  void createToolBar();
  void createWidget();

  void readIniFile();

private slots:
  void onActionNew();
  void onActionOpen();
  void onActionSave();
  void onActionBuild();
  void onActionClose();
  void onActionSetting();

private:
  QAction *m_pActionSave;
  QAction *m_pActionBuild;
  QAction *m_pActionClose;

  QReadDataObject *m_pObjReadData;
  QReadDataRosObject *m_pObjReadDataRos;
  QProjectObject *m_pObjProject;
  QDrawPathWidget *m_pWdgDrawPath;
  QDockWidget *m_pDockWdgPanel;
  QPanelWidget *m_pWdgPanel;
  QDockWidget *m_pDockWdgOutput;
  QTextBrowser *m_pTextBrowserOutput;
};

#endif // QEDITORMAINWINDOW_H
