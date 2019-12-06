#ifndef QSTUDIOWINDOW_H
#define QSTUDIOWINDOW_H

#include <QMainWindow>

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

public slots:

private:
  QCentralWidget *m_pWdgCentral;
};

#endif // QSTUDIOWINDOW_H
