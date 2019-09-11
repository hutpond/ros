/*******************************************************
 * Copyright(C) 2019 Deepblue
 * FileName: QDataDisplayQWidget.h
 * Author: liuzheng
 * Date: 2019/7/12
 * Description: 显示planning详细数据
********************************************************/
#ifndef Q_DATA_DISPLAY_WIDGET_H
#define Q_DATA_DISPLAY_WIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QTreeWidget;
class QDataDisplayWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QDataDisplayWidget(QWidget *parent = NULL);
  void setPlanningData(const debug_tool::ads_PlanningData4Debug &);

protected:
  virtual void resizeEvent(QResizeEvent *);

signals:

public slots:

private:
  QTreeWidget *m_pTreeWidget;
};

#endif // Q_DATA_DISPLAY_WIDGET_H
