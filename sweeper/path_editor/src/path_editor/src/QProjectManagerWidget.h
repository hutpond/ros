#ifndef QPROJECTMANAGERWIDGET_H
#define QPROJECTMANAGERWIDGET_H

#include <QWidget>
#include "MapDefines.h"

class QTreeWidget;
class QProjectObject;

class QProjectManagerWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QProjectManagerWidget(QProjectObject *, QWidget *parent = nullptr);

  void doUpdate();
  QSize sizeHint() const;

protected:
  void resizeEvent(QResizeEvent *);
  void contextMenuEvent(QContextMenuEvent *);

protected:

signals:
  void addBoundary(int, int, int);
  void operateSignal(MapOperation);

public slots:
  void onActionAddLane();
  void onActionAddBoundary();
  void onActionAddSignalSign();
  void onActionAddCrosswalk();
  void onActionAddStopSign();
  void onActionAddYieldSign();
  void onActionAddClearArea();
  void onActionAddSpeedBump();

private:
  QProjectObject *m_pObjProject;
  QTreeWidget *m_pTreeWidget;
};

#endif // QPROJECTMANAGERWIDGET_H
