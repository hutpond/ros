#ifndef QPROJECTMANAGERWIDGET_H
#define QPROJECTMANAGERWIDGET_H

#include <QWidget>

class QTreeWidget;
class QProjectObject;

class QProjectManagerWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QProjectManagerWidget(QProjectObject *, QWidget *parent = nullptr);

  void doUpdate();

protected:
  void resizeEvent(QResizeEvent *);
  void contextMenuEvent(QContextMenuEvent *);

signals:

public slots:
  void onActionAddLane();

private:
  QProjectObject *m_pObjProject;
  QTreeWidget *m_pTreeWidget;
};

#endif // QPROJECTMANAGERWIDGET_H
