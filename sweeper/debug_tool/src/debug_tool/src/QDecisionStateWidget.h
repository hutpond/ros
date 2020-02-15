#ifndef QDECISIONSTATEWIDGET_H
#define QDECISIONSTATEWIDGET_H

#include <QWidget>
#include "debug_tool/ads_PlanningData4Debug.h"

class QFrameTimeWidget;

class QDecisionStateWidget : public QWidget
{
  Q_OBJECT
public:
  explicit QDecisionStateWidget(QWidget *parent = nullptr);
  void setPlanningData(quint64, const debug_tool::ads_PlanningData4Debug &);
  void clearData();

protected:
  virtual void resizeEvent(QResizeEvent *) final;

signals:

public slots:

private:
  QFrameTimeWidget *m_pWdgFrameTime;
};

#endif // QDECISIONSTATEWIDGET_H
