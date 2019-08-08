/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDataDisplayDialog.h
 * Author: liuzheng
 * Date: 2019/7/12
 * Description: 显示planning详细数据
********************************************************/
#ifndef Q_DATA_DISPLAY_WIDGET_H
#define Q_DATA_DISPLAY_WIDGET_H

#include <QDialog>
#include "debug_tool/PlanningData4Debug.h"

class QTreeWidget;
class QDataDisplayDialog : public QDialog
{
  Q_OBJECT
public:
  explicit QDataDisplayDialog(QWidget *parent = NULL);
  void setPlanningData(const QString &, const debug_tool::PlanningData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *);

signals:

public slots:

private:
  QTreeWidget *m_pTreeWidget;
};

#endif // Q_DATA_DISPLAY_WIDGET_H
